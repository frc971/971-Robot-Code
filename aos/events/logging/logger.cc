#include "aos/events/logging/logger.h"

#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <vector>

#include "Eigen/Dense"
#include "absl/types/span.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "flatbuffers/flatbuffers.h"

DEFINE_bool(skip_missing_forwarding_entries, false,
            "If true, drop any forwarding entries with missing data.  If "
            "false, CHECK.");

DEFINE_bool(timestamps_to_csv, false,
            "If true, write all the time synchronization information to a set "
            "of CSV files in /tmp/.  This should only be needed when debugging "
            "time synchronization.");

namespace aos {
namespace logger {

namespace chrono = std::chrono;

Logger::Logger(DetachedBufferWriter *writer, EventLoop *event_loop,
               std::chrono::milliseconds polling_period)
    : Logger(std::make_unique<LocalLogNamer>(writer, event_loop->node()),
             event_loop, polling_period) {}

Logger::Logger(std::unique_ptr<LogNamer> log_namer, EventLoop *event_loop,
               std::chrono::milliseconds polling_period)
    : event_loop_(event_loop),
      log_namer_(std::move(log_namer)),
      timer_handler_(event_loop_->AddTimer([this]() { DoLogData(); })),
      polling_period_(polling_period) {
  VLOG(1) << "Starting logger for " << FlatbufferToJson(event_loop_->node());
  int channel_index = 0;
  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    FetcherStruct fs;
    const bool is_local =
        configuration::ChannelIsSendableOnNode(channel, event_loop_->node());

    const bool is_readable =
        configuration::ChannelIsReadableOnNode(channel, event_loop_->node());
    const bool log_message = configuration::ChannelMessageIsLoggedOnNode(
                                 channel, event_loop_->node()) &&
                             is_readable;

    const bool log_delivery_times =
        (event_loop_->node() == nullptr)
            ? false
            : configuration::ConnectionDeliveryTimeIsLoggedOnNode(
                  channel, event_loop_->node(), event_loop_->node());

    if (log_message || log_delivery_times) {
      fs.fetcher = event_loop->MakeRawFetcher(channel);
      VLOG(1) << "Logging channel "
              << configuration::CleanedChannelToString(channel);

      if (log_delivery_times) {
        VLOG(1) << "  Delivery times";
        fs.timestamp_writer = log_namer_->MakeTimestampWriter(channel);
      }
      if (log_message) {
        VLOG(1) << "  Data";
        fs.writer = log_namer_->MakeWriter(channel);
        if (!is_local) {
          fs.log_type = LogType::kLogRemoteMessage;
        }
      }
      fs.channel_index = channel_index;
      fs.written = false;
      fetchers_.emplace_back(std::move(fs));
    }
    ++channel_index;
  }

  // When things start, we want to log the header, then the most recent messages
  // available on each fetcher to capture the previous state, then start
  // polling.
  event_loop_->OnRun([this, polling_period]() {
    // Grab data from each channel right before we declare the log file started
    // so we can capture the latest message on each channel.  This lets us have
    // non periodic messages with configuration that now get logged.
    for (FetcherStruct &f : fetchers_) {
      f.written = !f.fetcher->Fetch();
    }

    // We need to pick a point in time to declare the log file "started".  This
    // starts here.  It needs to be after everything is fetched so that the
    // fetchers are all pointed at the most recent message before the start
    // time.
    monotonic_start_time_ = event_loop_->monotonic_now();
    realtime_start_time_ = event_loop_->realtime_now();
    last_synchronized_time_ = monotonic_start_time_;

    LOG(INFO) << "Logging node as " << FlatbufferToJson(event_loop_->node())
              << " start_time " << monotonic_start_time_;

    WriteHeader();

    timer_handler_->Setup(event_loop_->monotonic_now() + polling_period,
                          polling_period);
  });
}

// TODO(austin): Set the remote start time to the first time we see a remote
// message when we are logging those messages separate?  Need to signal what to
// do, or how to get a good timestamp.
void Logger::WriteHeader() {
  for (const Node *node : log_namer_->nodes()) {
    WriteHeader(node);
  }
}

void Logger::WriteHeader(const Node *node) {
  // Now write the header with this timestamp in it.
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  flatbuffers::Offset<aos::Configuration> configuration_offset =
      CopyFlatBuffer(event_loop_->configuration(), &fbb);

  flatbuffers::Offset<flatbuffers::String> string_offset =
      fbb.CreateString(network::GetHostname());

  flatbuffers::Offset<Node> node_offset;
  if (event_loop_->node() != nullptr) {
    node_offset = CopyFlatBuffer(node, &fbb);
  }

  aos::logger::LogFileHeader::Builder log_file_header_builder(fbb);

  log_file_header_builder.add_name(string_offset);

  // Only add the node if we are running in a multinode configuration.
  if (node != nullptr) {
    log_file_header_builder.add_node(node_offset);
  }

  log_file_header_builder.add_configuration(configuration_offset);
  // The worst case theoretical out of order is the polling period times 2.
  // One message could get logged right after the boundary, but be for right
  // before the next boundary.  And the reverse could happen for another
  // message.  Report back 3x to be extra safe, and because the cost isn't
  // huge on the read side.
  log_file_header_builder.add_max_out_of_order_duration(
      std::chrono::duration_cast<std::chrono::nanoseconds>(3 * polling_period_)
          .count());

  log_file_header_builder.add_monotonic_start_time(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          monotonic_start_time_.time_since_epoch())
          .count());
  log_file_header_builder.add_realtime_start_time(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          realtime_start_time_.time_since_epoch())
          .count());

  fbb.FinishSizePrefixed(log_file_header_builder.Finish());
  log_namer_->WriteHeader(&fbb, node);
}

void Logger::Rotate(DetachedBufferWriter *writer) {
  Rotate(std::make_unique<LocalLogNamer>(writer, event_loop_->node()));
}

void Logger::Rotate(std::unique_ptr<LogNamer> log_namer) {
  // Force data up until now to be written.
  DoLogData();

  // Swap the writer out, and re-write the header.
  log_namer_ = std::move(log_namer);

  // And then update the writers.
  for (FetcherStruct &f : fetchers_) {
    const Channel *channel =
        event_loop_->configuration()->channels()->Get(f.channel_index);
    if (f.timestamp_writer != nullptr) {
      f.timestamp_writer = log_namer_->MakeTimestampWriter(channel);
    }
    if (f.writer != nullptr) {
      f.writer = log_namer_->MakeWriter(channel);
    }
  }

  WriteHeader();
}

void Logger::DoLogData() {
  // We want to guarentee that messages aren't out of order by more than
  // max_out_of_order_duration.  To do this, we need sync points.  Every write
  // cycle should be a sync point.
  const monotonic_clock::time_point monotonic_now =
      event_loop_->monotonic_now();

  do {
    // Move the sync point up by at most polling_period.  This forces one sync
    // per iteration, even if it is small.
    last_synchronized_time_ =
        std::min(last_synchronized_time_ + polling_period_, monotonic_now);
    // Write each channel to disk, one at a time.
    for (FetcherStruct &f : fetchers_) {
      while (true) {
        if (f.written) {
          if (!f.fetcher->FetchNext()) {
            VLOG(2) << "No new data on "
                    << configuration::CleanedChannelToString(
                           f.fetcher->channel());
            break;
          } else {
            f.written = false;
          }
        }

        CHECK(!f.written);

        // TODO(james): Write tests to exercise this logic.
        if (f.fetcher->context().monotonic_event_time <
            last_synchronized_time_) {
          if (f.writer != nullptr) {
            // Write!
            flatbuffers::FlatBufferBuilder fbb(f.fetcher->context().size +
                                               max_header_size_);
            fbb.ForceDefaults(true);

            fbb.FinishSizePrefixed(PackMessage(&fbb, f.fetcher->context(),
                                               f.channel_index, f.log_type));

            VLOG(2) << "Writing data as node "
                    << FlatbufferToJson(event_loop_->node()) << " for channel "
                    << configuration::CleanedChannelToString(
                           f.fetcher->channel())
                    << " to " << f.writer->filename() << " data "
                    << FlatbufferToJson(
                           flatbuffers::GetSizePrefixedRoot<MessageHeader>(
                               fbb.GetBufferPointer()));

            max_header_size_ = std::max(
                max_header_size_, fbb.GetSize() - f.fetcher->context().size);
            f.writer->QueueSizedFlatbuffer(&fbb);
          }

          if (f.timestamp_writer != nullptr) {
            // And now handle timestamps.
            flatbuffers::FlatBufferBuilder fbb;
            fbb.ForceDefaults(true);

            fbb.FinishSizePrefixed(PackMessage(&fbb, f.fetcher->context(),
                                               f.channel_index,
                                               LogType::kLogDeliveryTimeOnly));

            VLOG(2) << "Writing timestamps as node "
                    << FlatbufferToJson(event_loop_->node()) << " for channel "
                    << configuration::CleanedChannelToString(
                           f.fetcher->channel())
                    << " to " << f.timestamp_writer->filename() << " timestamp "
                    << FlatbufferToJson(
                           flatbuffers::GetSizePrefixedRoot<MessageHeader>(
                               fbb.GetBufferPointer()));

            f.timestamp_writer->QueueSizedFlatbuffer(&fbb);
          }

          f.written = true;
        } else {
          break;
        }
      }
    }

    // If we missed cycles, we could be pretty far behind.  Spin until we are
    // caught up.
  } while (last_synchronized_time_ + polling_period_ < monotonic_now);
}

LogReader::LogReader(std::string_view filename,
                     const Configuration *replay_configuration)
    : LogReader(std::vector<std::string>{std::string(filename)},
                replay_configuration) {}

LogReader::LogReader(const std::vector<std::string> &filenames,
                     const Configuration *replay_configuration)
    : LogReader(std::vector<std::vector<std::string>>{filenames},
                replay_configuration) {}

LogReader::LogReader(const std::vector<std::vector<std::string>> &filenames,
                     const Configuration *replay_configuration)
    : filenames_(filenames),
      log_file_header_(ReadHeader(filenames[0][0])),
      replay_configuration_(replay_configuration) {
  MakeRemappedConfig();

  if (replay_configuration) {
    CHECK_EQ(configuration::MultiNode(configuration()),
             configuration::MultiNode(replay_configuration))
        << ": Log file and replay config need to both be multi or single node.";
  }

  if (!configuration::MultiNode(configuration())) {
    states_.emplace_back(std::make_unique<State>());
    State *state = states_[0].get();

    state->channel_merger = std::make_unique<ChannelMerger>(filenames);
  } else {
    if (replay_configuration) {
      CHECK_EQ(logged_configuration()->nodes()->size(),
               replay_configuration->nodes()->size())
          << ": Log file and replay config need to have matching nodes lists.";
      for (const Node *node : *logged_configuration()->nodes()) {
        if (configuration::GetNode(replay_configuration, node) == nullptr) {
          LOG(FATAL)
              << "Found node " << FlatbufferToJson(node)
              << " in logged config that is not present in the replay config.";
        }
      }
    }
    states_.resize(configuration()->nodes()->size());
  }
}

LogReader::~LogReader() {
  if (event_loop_factory_unique_ptr_) {
    Deregister();
  } else if (event_loop_factory_ != nullptr) {
    LOG(FATAL) << "Must call Deregister before the SimulatedEventLoopFactory "
                  "is destroyed";
  }
  if (offset_fp_ != nullptr) {
    fclose(offset_fp_);
  }
  // Zero out some buffers. It's easy to do use-after-frees on these, so make it
  // more obvious.
  if (remapped_configuration_buffer_) {
    remapped_configuration_buffer_->Wipe();
  }
  log_file_header_.Wipe();
}

const Configuration *LogReader::logged_configuration() const {
  return log_file_header_.message().configuration();
}

const Configuration *LogReader::configuration() const {
  return remapped_configuration_;
}

std::vector<const Node *> LogReader::Nodes() const {
  // Because the Node pointer will only be valid if it actually points to memory
  // owned by remapped_configuration_, we need to wait for the
  // remapped_configuration_ to be populated before accessing it.
  //
  // Also, note, that when ever a map is changed, the nodes in here are
  // invalidated.
  CHECK(remapped_configuration_ != nullptr)
      << ": Need to call Register before the node() pointer will be valid.";
  return configuration::GetNodes(remapped_configuration_);
}

monotonic_clock::time_point LogReader::monotonic_start_time(const Node *node) {
  State *state =
      states_[configuration::GetNodeIndex(configuration(), node)].get();
  CHECK(state != nullptr) << ": Unknown node " << FlatbufferToJson(node);

  return state->channel_merger->monotonic_start_time();
}

realtime_clock::time_point LogReader::realtime_start_time(const Node *node) {
  State *state =
      states_[configuration::GetNodeIndex(configuration(), node)].get();
  CHECK(state != nullptr) << ": Unknown node " << FlatbufferToJson(node);

  return state->channel_merger->realtime_start_time();
}

void LogReader::Register() {
  event_loop_factory_unique_ptr_ =
      std::make_unique<SimulatedEventLoopFactory>(configuration());
  Register(event_loop_factory_unique_ptr_.get());
}

void LogReader::Register(SimulatedEventLoopFactory *event_loop_factory) {
  event_loop_factory_ = event_loop_factory;

  for (const Node *node : configuration::GetNodes(configuration())) {
    const size_t node_index =
        configuration::GetNodeIndex(configuration(), node);
    states_[node_index] = std::make_unique<State>();
    State *state = states_[node_index].get();

    state->channel_merger = std::make_unique<ChannelMerger>(filenames_);

    state->node_event_loop_factory =
        event_loop_factory_->GetNodeEventLoopFactory(node);
    state->event_loop_unique_ptr =
        event_loop_factory->MakeEventLoop("log_reader", node);

    Register(state->event_loop_unique_ptr.get());
  }
  if (live_nodes_ == 0) {
    LOG(FATAL)
        << "Don't have logs from any of the nodes in the replay config--are "
           "you sure that the replay config matches the original config?";
  }

  // We need to now seed our per-node time offsets and get everything set up to
  // run.
  const size_t num_nodes = !configuration::MultiNode(logged_configuration())
                               ? 1u
                               : logged_configuration()->nodes()->size();

  // It is easiest to solve for per node offsets with a matrix rather than
  // trying to solve the equations by hand.  So let's get after it.
  //
  // Now, build up the map matrix.
  //
  // sample_matrix_ = map_matrix_ * offset_matrix_
  map_matrix_ = Eigen::MatrixXd::Zero(filters_.size() + 1, num_nodes);

  sample_matrix_ = Eigen::VectorXd::Zero(filters_.size() + 1);
  offset_matrix_ = Eigen::VectorXd::Zero(num_nodes);

  // And the base offset matrix, which will be a copy of the initial offset
  // matrix.
  base_offset_matrix_ =
      Eigen::Matrix<std::chrono::nanoseconds, Eigen::Dynamic, 1>::Zero(
          num_nodes);

  // All offsets should sum to 0.  Add that as the first constraint in our least
  // squares.
  map_matrix_.row(0).setOnes();

  {
    // Now, add the a - b -> sample elements.
    size_t i = 1;
    for (std::pair<const std::tuple<const Node *, const Node *>,
                   message_bridge::ClippedAverageFilter> &filter : filters_) {
      const Node *const node_a = std::get<0>(filter.first);
      const Node *const node_b = std::get<1>(filter.first);

      const size_t node_a_index =
          configuration::GetNodeIndex(configuration(), node_a);
      const size_t node_b_index =
          configuration::GetNodeIndex(configuration(), node_b);

      // +a
      map_matrix_(i, node_a_index) = 1.0;
      // -b
      map_matrix_(i, node_b_index) = -1.0;

      // -> sample
      filter.second.set_sample_pointer(&sample_matrix_(i, 0));

      ++i;
    }
  }

  // Rank of the map matrix tells you if all the nodes are in communication with
  // each other, which tells you if the offsets are observable.
  const size_t connected_nodes =
      Eigen::FullPivLU<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(
          map_matrix_)
          .rank();

  // We don't need to support isolated nodes until someone has a real use case.
  CHECK_EQ(connected_nodes, num_nodes)
      << ": There is a node which isn't communicating with the rest.";

  // Now, iterate through all the timestamps from all the nodes and seed
  // everything.
  for (std::unique_ptr<State> &state : states_) {
    for (size_t i = 0; i < logged_configuration()->channels()->size(); ++i) {
      TimestampMerger::DeliveryTimestamp timestamp =
          state->channel_merger->OldestTimestampForChannel(i);
      if (timestamp.monotonic_event_time != monotonic_clock::min_time) {
        CHECK(state->MaybeUpdateTimestamp(timestamp, i));
      }
    }
  }

  // Make sure all the samples have been seeded.
  for (int i = 1; i < sample_matrix_.cols(); ++i) {
    // The seeding logic is pretty basic right now because we don't have great
    // use cases yet.  It wants to see data from every node.  Blow up for now,
    // and once we have a reason to do something different, update this logic.
    // Maybe read further in the log file?  Or seed off the realtime time?
    CHECK_NE(sample_matrix_(i, 0), 0.0)
        << ": Sample " << i << " is not seeded.";
  }

  // And solve.
  offset_matrix_ = SolveOffsets();

  // Save off the base offsets so we can work in deltas from here out.  That
  // will significantly simplify the numerical precision problems.
  for (size_t i = 0; i < num_nodes; ++i) {
    base_offset_matrix_(i, 0) =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(offset_matrix_(i, 0)));
  }

  {
    // Shift everything so we never could (reasonably) require the distributed
    // clock to have a large backwards jump in time.  This makes it so the boot
    // time on the node up the longest will essentially start matching the
    // distributed clock.
    const chrono::nanoseconds offset = -base_offset_matrix_.maxCoeff();
    for (int i = 0; i < base_offset_matrix_.rows(); ++i) {
      base_offset_matrix_(i, 0) += offset;
    }
  }

  {
    // Re-compute the samples and setup all the filters so that they
    // subtract this base offset.

    size_t i = 1;
    for (std::pair<const std::tuple<const Node *, const Node *>,
                   message_bridge::ClippedAverageFilter> &filter : filters_) {
      CHECK(filter.second.sample_pointer() == &sample_matrix_(i, 0));

      const Node *const node_a = std::get<0>(filter.first);
      const Node *const node_b = std::get<1>(filter.first);

      const size_t node_a_index =
          configuration::GetNodeIndex(configuration(), node_a);
      const size_t node_b_index =
          configuration::GetNodeIndex(configuration(), node_b);

      filter.second.set_base_offset(base_offset_matrix_(node_a_index) -
                                    base_offset_matrix_(node_b_index));

      ++i;
    }
  }

  // Now, iterate again through all the offsets now that we have set the base
  // offset to something sane.  This will seed everything with an accurate
  // initial offset.
  for (std::unique_ptr<State> &state : states_) {
    for (size_t i = 0; i < logged_configuration()->channels()->size(); ++i) {
      TimestampMerger::DeliveryTimestamp timestamp =
          state->channel_merger->OldestTimestampForChannel(i);
      if (timestamp.monotonic_event_time != monotonic_clock::min_time) {
        CHECK(state->MaybeUpdateTimestamp(timestamp, i));
      }
    }
  }

  UpdateOffsets();

  // We want to start the log file at the last start time of the log files from
  // all the nodes.  Compute how long each node's simulation needs to run to
  // move time to this point.
  distributed_clock::time_point start_time = distributed_clock::min_time;

  for (std::unique_ptr<State> &state : states_) {
    // Setup the realtime clock to have something sane in it now.
    state->node_event_loop_factory->SetRealtimeOffset(
        state->channel_merger->monotonic_start_time(),
        state->channel_merger->realtime_start_time());
    // And start computing the start time on the distributed clock now that that
    // works.
    start_time = std::max(start_time,
                          state->node_event_loop_factory->ToDistributedClock(
                              state->channel_merger->monotonic_start_time()));
  }
  CHECK_GE(start_time, distributed_clock::epoch());

  // Forwarding is tracked per channel.  If it is enabled, we want to turn it
  // off.  Otherwise messages replayed will get forwarded across to the other
  // nodes, and also replayed on the other nodes.  This may not satisfy all our
  // users, but it'll start the discussion.
  if (configuration::MultiNode(event_loop_factory_->configuration())) {
    for (size_t i = 0; i < logged_configuration()->channels()->size(); ++i) {
      const Channel *channel = logged_configuration()->channels()->Get(i);
      const Node *node = configuration::GetNode(
          configuration(), channel->source_node()->string_view());

      State *state =
          states_[configuration::GetNodeIndex(configuration(), node)].get();

      const Channel *remapped_channel =
          RemapChannel(state->event_loop, channel);

      event_loop_factory_->DisableForwarding(remapped_channel);
    }

    // If we are replaying a log, we don't want a bunch of redundant messages
    // from both the real message bridge and simulated message bridge.
    event_loop_factory_->DisableStatistics();
  }

  // While we are starting the system up, we might be relying on matching data
  // to timestamps on log files where the timestamp log file starts before the
  // data.  In this case, it is reasonable to expect missing data.
  ignore_missing_data_ = true;
  VLOG(1) << "Running until start time: " << start_time;
  event_loop_factory_->RunFor(start_time.time_since_epoch());
  VLOG(1) << "At start time";
  // Now that we are running for real, missing data means that the log file is
  // corrupted or went wrong.
  ignore_missing_data_ = false;
}

void LogReader::UpdateOffsets() {
  // TODO(austin): Evaluate less accurate inverses.  We might be able to
  // do some tricks to keep the accuracy up.
  offset_matrix_ = SolveOffsets();

  size_t node_index = 0;
  for (std::unique_ptr<State> &state : states_) {
    state->node_event_loop_factory->SetDistributedOffset(-offset(node_index),
                                                         1.0);
    ++node_index;
  }
}

std::tuple<message_bridge::ClippedAverageFilter *, bool> LogReader::GetFilter(
    const Node *node_a, const Node *node_b) {
  CHECK_NE(node_a, node_b);
  CHECK_EQ(configuration::GetNode(configuration(), node_a), node_a);
  CHECK_EQ(configuration::GetNode(configuration(), node_b), node_b);

  if (node_a > node_b) {
    return std::make_pair(std::get<0>(GetFilter(node_b, node_a)), false);
  }

  auto tuple = std::make_tuple(node_a, node_b);

  auto it = filters_.find(tuple);

  if (it == filters_.end()) {
    auto &x = filters_
                  .insert(std::make_pair(
                      tuple, message_bridge::ClippedAverageFilter()))
                  .first->second;
    if (FLAGS_timestamps_to_csv) {
      std::string fwd_name =
          absl::StrCat("/tmp/timestamp_", node_a->name()->string_view(), "_",
                       node_b->name()->string_view());
      x.SetFwdCsvFileName(fwd_name);
      std::string rev_name =
          absl::StrCat("/tmp/timestamp_", node_b->name()->string_view(), "_",
                       node_a->name()->string_view());
      x.SetRevCsvFileName(rev_name);
    }

    return std::make_tuple(&x, true);
  } else {
    return std::make_tuple(&(it->second), true);
  }
}

bool LogReader::State::MaybeUpdateTimestamp(
    const TimestampMerger::DeliveryTimestamp &channel_timestamp,
    int channel_index) {
  if (channel_timestamp.monotonic_remote_time == monotonic_clock::min_time) {
    return false;
  }

  // Got a forwarding timestamp!
  CHECK(std::get<0>(filters[channel_index]) != nullptr);

  // Call the correct method depending on if we are the forward or reverse
  // direction here.
  if (std::get<1>(filters[channel_index])) {
    std::get<0>(filters[channel_index])
        ->FwdSample(channel_timestamp.monotonic_event_time,
                    channel_timestamp.monotonic_event_time -
                        channel_timestamp.monotonic_remote_time);
  } else {
    std::get<0>(filters[channel_index])
        ->RevSample(channel_timestamp.monotonic_event_time,
                    channel_timestamp.monotonic_event_time -
                        channel_timestamp.monotonic_remote_time);
  }
  return true;
}

void LogReader::Register(EventLoop *event_loop) {
  State *state =
      states_[configuration::GetNodeIndex(configuration(), event_loop->node())]
          .get();

  state->event_loop = event_loop;

  // We don't run timing reports when trying to print out logged data, because
  // otherwise we would end up printing out the timing reports themselves...
  // This is only really relevant when we are replaying into a simulation.
  event_loop->SkipTimingReport();
  event_loop->SkipAosLog();

  const bool has_data = state->channel_merger->SetNode(event_loop->node());

  state->channels.resize(logged_configuration()->channels()->size());
  state->filters.resize(state->channels.size());

  state->channel_target_event_loop_factory.resize(state->channels.size());

  for (size_t i = 0; i < state->channels.size(); ++i) {
    const Channel *channel =
        RemapChannel(event_loop, logged_configuration()->channels()->Get(i));

    state->channels[i] = event_loop->MakeRawSender(channel);

    state->filters[i] = std::make_tuple(nullptr, false);

    if (!configuration::ChannelIsSendableOnNode(channel, event_loop->node()) &&
        configuration::ChannelIsReadableOnNode(channel, event_loop->node())) {
      const Node *target_node = configuration::GetNode(
          event_loop->configuration(), channel->source_node()->string_view());
      state->filters[i] = GetFilter(event_loop->node(), target_node);

      if (event_loop_factory_ != nullptr) {
        state->channel_target_event_loop_factory[i] =
            event_loop_factory_->GetNodeEventLoopFactory(target_node);
      }
    }
  }

  // If we didn't find any log files with data in them, we won't ever get a
  // callback or be live.  So skip the rest of the setup.
  if (!has_data) {
    return;
  }

  state->timer_handler = event_loop->AddTimer([this, state]() {
    if (state->channel_merger->OldestMessage() == monotonic_clock::max_time) {
      --live_nodes_;
      VLOG(1) << "Node down!";
      if (live_nodes_ == 0) {
        event_loop_factory_->Exit();
      }
      return;
    }
    bool update_offsets = false;
    TimestampMerger::DeliveryTimestamp channel_timestamp;
    int channel_index;
    FlatbufferVector<MessageHeader> channel_data =
        FlatbufferVector<MessageHeader>::Empty();

    std::tie(channel_timestamp, channel_index, channel_data) =
        state->channel_merger->PopOldest();

    const monotonic_clock::time_point monotonic_now =
        state->event_loop->context().monotonic_event_time;
    CHECK(monotonic_now == channel_timestamp.monotonic_event_time)
        << ": " << FlatbufferToJson(state->event_loop->node()) << " Now "
        << monotonic_now << " trying to send "
        << channel_timestamp.monotonic_event_time << " failure "
        << state->channel_merger->DebugString();

    if (channel_timestamp.monotonic_event_time >
            state->channel_merger->monotonic_start_time() ||
        event_loop_factory_ != nullptr) {
      if ((!ignore_missing_data_ && !FLAGS_skip_missing_forwarding_entries &&
           !state->channel_merger->at_end()) ||
          channel_data.message().data() != nullptr) {
        CHECK(channel_data.message().data() != nullptr)
            << ": Got a message without data.  Forwarding entry which was "
               "not matched?  Use --skip_missing_forwarding_entries to ignore "
               "this.";

        if (state->MaybeUpdateTimestamp(channel_timestamp, channel_index)) {
          // Confirm that the message was sent on the sending node before the
          // destination node (this node).  As a proxy, do this by making sure
          // that time on the source node is past when the message was sent.
          CHECK_LT(channel_timestamp.monotonic_remote_time,
                   state->channel_target_event_loop_factory[channel_index]
                       ->monotonic_now());

          update_offsets = true;

          if (FLAGS_timestamps_to_csv) {
            if (offset_fp_ == nullptr) {
              offset_fp_ = fopen("/tmp/offsets.csv", "w");
              fprintf(
                  offset_fp_,
                  "# time_since_start, offset node 0, offset node 1, ...\n");
              first_time_ = channel_timestamp.realtime_event_time;
            }

            fprintf(offset_fp_, "%.9f",
                    std::chrono::duration_cast<std::chrono::duration<double>>(
                        channel_timestamp.realtime_event_time - first_time_)
                        .count());
            for (int i = 0; i < base_offset_matrix_.rows(); ++i) {
              fprintf(
                  offset_fp_, ", %.9f",
                  offset_matrix_(i, 0) +
                      std::chrono::duration_cast<std::chrono::duration<double>>(
                          base_offset_matrix_(i, 0))
                          .count());
            }
            fprintf(offset_fp_, "\n");
          }

        } else {
          CHECK(std::get<0>(state->filters[channel_index]) == nullptr);
        }

        // If we have access to the factory, use it to fix the realtime time.
        if (state->node_event_loop_factory != nullptr) {
          state->node_event_loop_factory->SetRealtimeOffset(
              channel_timestamp.monotonic_event_time,
              channel_timestamp.realtime_event_time);
        }

        state->channels[channel_index]->Send(
            channel_data.message().data()->Data(),
            channel_data.message().data()->size(),
            channel_timestamp.monotonic_remote_time,
            channel_timestamp.realtime_remote_time,
            channel_timestamp.remote_queue_index);
      } else if (state->channel_merger->at_end()) {
        // We are at the end of the log file and found missing data.  Finish
        // reading the rest of the log file and call it quits.  We don't want to
        // replay partial data.
        while (state->channel_merger->OldestMessage() !=
               monotonic_clock::max_time) {
          state->channel_merger->PopOldest();
        }
      }

    } else {
      LOG(WARNING)
          << "Not sending data from before the start of the log file. "
          << channel_timestamp.monotonic_event_time.time_since_epoch().count()
          << " start " << monotonic_start_time().time_since_epoch().count()
          << " " << FlatbufferToJson(channel_data);
    }

    const monotonic_clock::time_point next_time =
        state->channel_merger->OldestMessage();
    if (next_time != monotonic_clock::max_time) {
      state->timer_handler->Setup(next_time);
    } else {
      // Set a timer up immediately after now to die. If we don't do this, then
      // the senders waiting on the message we just read will never get called.
      if (event_loop_factory_ != nullptr) {
        state->timer_handler->Setup(monotonic_now +
                                    event_loop_factory_->send_delay() +
                                    std::chrono::nanoseconds(1));
      }
    }

    // Once we make this call, the current time changes.  So do everything which
    // involves time before changing it.  That especially includes sending the
    // message.
    if (update_offsets) {
      UpdateOffsets();
    }
  });

  ++live_nodes_;

  if (state->channel_merger->OldestMessage() != monotonic_clock::max_time) {
    event_loop->OnRun([state]() {
      state->timer_handler->Setup(state->channel_merger->OldestMessage());
    });
  }
}

void LogReader::Deregister() {
  // Make sure that things get destroyed in the correct order, rather than
  // relying on getting the order correct in the class definition.
  for (std::unique_ptr<State> &state : states_) {
    for (size_t i = 0; i < state->channels.size(); ++i) {
      state->channels[i].reset();
    }
    state->event_loop_unique_ptr.reset();
    state->event_loop = nullptr;
    state->node_event_loop_factory = nullptr;
  }

  event_loop_factory_unique_ptr_.reset();
  event_loop_factory_ = nullptr;
}

void LogReader::RemapLoggedChannel(std::string_view name, std::string_view type,
                                   std::string_view add_prefix) {
  for (size_t ii = 0; ii < logged_configuration()->channels()->size(); ++ii) {
    const Channel *const channel = logged_configuration()->channels()->Get(ii);
    if (channel->name()->str() == name &&
        channel->type()->string_view() == type) {
      CHECK_EQ(0u, remapped_channels_.count(ii))
          << "Already remapped channel "
          << configuration::CleanedChannelToString(channel);
      remapped_channels_[ii] = std::string(add_prefix) + std::string(name);
      VLOG(1) << "Remapping channel "
              << configuration::CleanedChannelToString(channel)
              << " to have name " << remapped_channels_[ii];
      MakeRemappedConfig();
      return;
    }
  }
  LOG(FATAL) << "Unabled to locate channel with name " << name << " and type "
             << type;
}

void LogReader::MakeRemappedConfig() {
  for (std::unique_ptr<State> &state : states_) {
    if (state) {
      CHECK(!state->event_loop)
          << ": Can't change the mapping after the events are scheduled.";
    }
  }

  // If no remapping occurred and we are using the original config, then there
  // is nothing interesting to do here.
  if (remapped_channels_.empty() && replay_configuration_ == nullptr) {
    remapped_configuration_ = logged_configuration();
    return;
  }
  // Config to copy Channel definitions from. Use the specified
  // replay_configuration_ if it has been provided.
  const Configuration *const base_config = replay_configuration_ == nullptr
                                               ? logged_configuration()
                                               : replay_configuration_;
  // The remapped config will be identical to the base_config, except that it
  // will have a bunch of extra channels in the channel list, which are exact
  // copies of the remapped channels, but with different names.
  // Because the flatbuffers API is a pain to work with, this requires a bit of
  // a song-and-dance to get copied over.
  // The order of operations is to:
  // 1) Make a flatbuffer builder for a config that will just contain a list of
  //    the new channels that we want to add.
  // 2) For each channel that we are remapping:
  //    a) Make a buffer/builder and construct into it a Channel table that only
  //       contains the new name for the channel.
  //    b) Merge the new channel with just the name into the channel that we are
  //       trying to copy, built in the flatbuffer builder made in 1. This gives
  //       us the new channel definition that we need.
  // 3) Using this list of offsets, build the Configuration of just new
  //    Channels.
  // 4) Merge the Configuration with the new Channels into the base_config.
  // 5) Call MergeConfiguration() on that result to give MergeConfiguration a
  //    chance to sanitize the config.

  // This is the builder that we use for the config containing all the new
  // channels.
  flatbuffers::FlatBufferBuilder new_config_fbb;
  new_config_fbb.ForceDefaults(true);
  std::vector<flatbuffers::Offset<Channel>> channel_offsets;
  for (auto &pair : remapped_channels_) {
    // This is the builder that we use for creating the Channel with just the
    // new name.
    flatbuffers::FlatBufferBuilder new_name_fbb;
    new_name_fbb.ForceDefaults(true);
    const flatbuffers::Offset<flatbuffers::String> name_offset =
        new_name_fbb.CreateString(pair.second);
    ChannelBuilder new_name_builder(new_name_fbb);
    new_name_builder.add_name(name_offset);
    new_name_fbb.Finish(new_name_builder.Finish());
    const FlatbufferDetachedBuffer<Channel> new_name = new_name_fbb.Release();
    // Retrieve the channel that we want to copy, confirming that it is actually
    // present in base_config.
    const Channel *const base_channel = CHECK_NOTNULL(configuration::GetChannel(
        base_config, logged_configuration()->channels()->Get(pair.first), "",
        nullptr));
    // Actually create the new channel and put it into the vector of Offsets
    // that we will use to create the new Configuration.
    channel_offsets.emplace_back(MergeFlatBuffers<Channel>(
        reinterpret_cast<const flatbuffers::Table *>(base_channel),
        reinterpret_cast<const flatbuffers::Table *>(&new_name.message()),
        &new_config_fbb));
  }
  // Create the Configuration containing the new channels that we want to add.
  const auto new_name_vector_offsets =
      new_config_fbb.CreateVector(channel_offsets);
  ConfigurationBuilder new_config_builder(new_config_fbb);
  new_config_builder.add_channels(new_name_vector_offsets);
  new_config_fbb.Finish(new_config_builder.Finish());
  const FlatbufferDetachedBuffer<Configuration> new_name_config =
      new_config_fbb.Release();
  // Merge the new channels configuration into the base_config, giving us the
  // remapped configuration.
  remapped_configuration_buffer_ =
      std::make_unique<FlatbufferDetachedBuffer<Configuration>>(
          MergeFlatBuffers<Configuration>(base_config,
                                          &new_name_config.message()));
  // Call MergeConfiguration to deal with sanitizing the config.
  remapped_configuration_buffer_ =
      std::make_unique<FlatbufferDetachedBuffer<Configuration>>(
          configuration::MergeConfiguration(*remapped_configuration_buffer_));

  remapped_configuration_ = &remapped_configuration_buffer_->message();
}

const Channel *LogReader::RemapChannel(const EventLoop *event_loop,
                                       const Channel *channel) {
  std::string_view channel_name = channel->name()->string_view();
  std::string_view channel_type = channel->type()->string_view();
  const int channel_index =
      configuration::ChannelIndex(logged_configuration(), channel);
  // If the channel is remapped, find the correct channel name to use.
  if (remapped_channels_.count(channel_index) > 0) {
    VLOG(3) << "Got remapped channel on "
            << configuration::CleanedChannelToString(channel);
    channel_name = remapped_channels_[channel_index];
  }

  VLOG(2) << "Going to remap channel " << channel_name << " " << channel_type;
  const Channel *remapped_channel = configuration::GetChannel(
      event_loop->configuration(), channel_name, channel_type,
      event_loop->name(), event_loop->node());

  CHECK(remapped_channel != nullptr)
      << ": Unable to send {\"name\": \"" << channel_name << "\", \"type\": \""
      << channel_type << "\"} because it is not in the provided configuration.";

  return remapped_channel;
}

}  // namespace logger
}  // namespace aos
