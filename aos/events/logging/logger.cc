#include "aos/events/logging/logger.h"

#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <vector>

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

namespace aos {
namespace logger {

namespace chrono = std::chrono;

Logger::Logger(DetachedBufferWriter *writer, EventLoop *event_loop,
               std::chrono::milliseconds polling_period)
    : event_loop_(event_loop),
      writer_(writer),
      timer_handler_(event_loop_->AddTimer([this]() { DoLogData(); })),
      polling_period_(polling_period) {
  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    FetcherStruct fs;
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
        if (log_message) {
          VLOG(1) << "  Logging message and delivery times";
          fs.log_type = LogType::kLogMessageAndDeliveryTime;
        } else {
          VLOG(1) << "  Logging delivery times only";
          fs.log_type = LogType::kLogDeliveryTimeOnly;
        }
      } else {
        // We don't have a particularly great use case right now for logging a
        // forwarded message, but either not logging the delivery times, or
        // logging them on another node.  Fail rather than produce bad results.
        CHECK(configuration::ChannelIsSendableOnNode(channel,
                                                     event_loop_->node()))
            << ": Logger only knows how to log remote messages with "
               "forwarding timestamps.";
        VLOG(1) << "  Logging message only";
        fs.log_type = LogType::kLogMessage;
      }
    }

    fs.written = false;
    fetchers_.emplace_back(std::move(fs));
  }

  // When things start, we want to log the header, then the most recent messages
  // available on each fetcher to capture the previous state, then start
  // polling.
  event_loop_->OnRun([this, polling_period]() {
    // Grab data from each channel right before we declare the log file started
    // so we can capture the latest message on each channel.  This lets us have
    // non periodic messages with configuration that now get logged.
    for (FetcherStruct &f : fetchers_) {
      if (f.fetcher.get() != nullptr) {
        f.written = !f.fetcher->Fetch();
      }
    }

    // We need to pick a point in time to declare the log file "started".  This
    // starts here.  It needs to be after everything is fetched so that the
    // fetchers are all pointed at the most recent message before the start
    // time.
    monotonic_start_time_ = event_loop_->monotonic_now();
    realtime_start_time_ = event_loop_->realtime_now();
    last_synchronized_time_ = monotonic_start_time_;

    LOG(INFO) << "Logging node as " << FlatbufferToJson(event_loop_->node());

    WriteHeader();

    timer_handler_->Setup(event_loop_->monotonic_now() + polling_period,
                          polling_period);
  });
}

void Logger::WriteHeader() {
  // Now write the header with this timestamp in it.
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(1);

  flatbuffers::Offset<aos::Configuration> configuration_offset =
      CopyFlatBuffer(event_loop_->configuration(), &fbb);

  flatbuffers::Offset<flatbuffers::String> string_offset =
      fbb.CreateString(network::GetHostname());

  flatbuffers::Offset<Node> node_offset;
  if (event_loop_->node() != nullptr) {
    node_offset = CopyFlatBuffer(event_loop_->node(), &fbb);
  }

  aos::logger::LogFileHeader::Builder log_file_header_builder(fbb);

  log_file_header_builder.add_name(string_offset);

  // Only add the node if we are running in a multinode configuration.
  if (event_loop_->node() != nullptr) {
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
  writer_->QueueSizedFlatbuffer(&fbb);
}

void Logger::Rotate(DetachedBufferWriter *writer) {
  // Force data up until now to be written.
  DoLogData();

  // Swap the writer out, and re-write the header.
  writer_ = writer;
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
    size_t channel_index = 0;
    // Write each channel to disk, one at a time.
    for (FetcherStruct &f : fetchers_) {
      // Skip any channels which we aren't supposed to log.
      if (f.fetcher.get() != nullptr) {
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
            // Write!
            flatbuffers::FlatBufferBuilder fbb(f.fetcher->context().size +
                                               max_header_size_);
            fbb.ForceDefaults(1);

            fbb.FinishSizePrefixed(PackMessage(&fbb, f.fetcher->context(),
                                               channel_index, f.log_type));

            VLOG(2) << "Writing data for channel "
                    << configuration::CleanedChannelToString(
                           f.fetcher->channel());

            max_header_size_ = std::max(
                max_header_size_, fbb.GetSize() - f.fetcher->context().size);
            writer_->QueueSizedFlatbuffer(&fbb);

            f.written = true;
          } else {
            break;
          }
        }
      }

      ++channel_index;
    }

    CHECK_EQ(channel_index, fetchers_.size());

    // If we missed cycles, we could be pretty far behind.  Spin until we are
    // caught up.
  } while (last_synchronized_time_ + polling_period_ < monotonic_now);

  writer_->Flush();
}

LogReader::LogReader(std::string_view filename,
                     const Configuration *replay_configuration)
    : LogReader(std::vector<std::string>{std::string(filename)},
                replay_configuration) {}

LogReader::LogReader(const std::vector<std::string> &filenames,
                     const Configuration *replay_configuration)
    : sorted_message_reader_(filenames),
      replay_configuration_(replay_configuration) {
  channels_.resize(logged_configuration()->channels()->size());
  MakeRemappedConfig();
}

LogReader::~LogReader() { Deregister(); }

const Configuration *LogReader::logged_configuration() const {
  return sorted_message_reader_.configuration();
}

const Configuration *LogReader::configuration() const {
  return remapped_configuration_;
}

const Node *LogReader::node() const {
  // Because the Node pointer will only be valid if it actually points to memory
  // owned by remapped_configuration_, we need to wait for the
  // remapped_configuration_ to be populated before accessing it.
  CHECK(remapped_configuration_ != nullptr)
      << ": Need to call Register before the node() pointer will be valid.";
  if (sorted_message_reader_.node() == nullptr) {
    return nullptr;
  }
  return configuration::GetNode(
      configuration(), sorted_message_reader_.node()->name()->string_view());
}

monotonic_clock::time_point LogReader::monotonic_start_time() {
  return sorted_message_reader_.monotonic_start_time();
}

realtime_clock::time_point LogReader::realtime_start_time() {
  return sorted_message_reader_.realtime_start_time();
}

void LogReader::Register() {
  event_loop_factory_unique_ptr_ =
      std::make_unique<SimulatedEventLoopFactory>(configuration(), node());
  Register(event_loop_factory_unique_ptr_.get());
}

void LogReader::Register(SimulatedEventLoopFactory *event_loop_factory) {
  event_loop_factory_ = event_loop_factory;
  event_loop_unique_ptr_ = event_loop_factory_->MakeEventLoop("log_reader");
  // We don't run timing reports when trying to print out logged data, because
  // otherwise we would end up printing out the timing reports themselves...
  // This is only really relevant when we are replaying into a simulation.
  event_loop_unique_ptr_->SkipTimingReport();

  Register(event_loop_unique_ptr_.get());
  event_loop_factory_->RunFor(monotonic_start_time() -
                              event_loop_->monotonic_now());
}

void LogReader::Register(EventLoop *event_loop) {
  event_loop_ = event_loop;

  // Otherwise we replay the timing report and try to resend it...
  event_loop_->SkipTimingReport();

  for (size_t i = 0; i < channels_.size(); ++i) {
    const Channel *const original_channel =
        logged_configuration()->channels()->Get(i);

    std::string_view channel_name = original_channel->name()->string_view();
    std::string_view channel_type = original_channel->type()->string_view();
    // If the channel is remapped, find the correct channel name to use.
    if (remapped_channels_.count(i) > 0) {
      VLOG(2) << "Got remapped channel on "
              << configuration::CleanedChannelToString(original_channel);
      channel_name = remapped_channels_[i];
    }

    VLOG(1) << "Going to remap channel " << channel_name << " " << channel_type;
    const Channel *channel = configuration::GetChannel(
        event_loop_->configuration(), channel_name, channel_type,
        event_loop_->name(), event_loop_->node());

    CHECK(channel != nullptr)
        << ": Unable to send {\"name\": \"" << channel_name
        << "\", \"type\": \"" << channel_type
        << "\"} because it is not in the provided configuration.";

    channels_[i] = event_loop_->MakeRawSender(channel);
  }

  timer_handler_ = event_loop_->AddTimer([this]() {
    if (sorted_message_reader_.active_channel_count() == 0u) {
      event_loop_factory_->Exit();
      return;
    }
    monotonic_clock::time_point channel_timestamp;
    int channel_index;
    FlatbufferVector<MessageHeader> channel_data =
        FlatbufferVector<MessageHeader>::Empty();

    std::tie(channel_timestamp, channel_index, channel_data) =
        sorted_message_reader_.PopOldestChannel();

    const monotonic_clock::time_point monotonic_now =
        event_loop_->context().monotonic_event_time;
    CHECK(monotonic_now == channel_timestamp)
        << ": Now " << monotonic_now.time_since_epoch().count()
        << " trying to send " << channel_timestamp.time_since_epoch().count();

    if (channel_timestamp > monotonic_start_time() ||
        event_loop_factory_ != nullptr) {
      if (!FLAGS_skip_missing_forwarding_entries ||
          channel_data.message().data() != nullptr) {
        CHECK(channel_data.message().data() != nullptr)
            << ": Got a message without data.  Forwarding entry which was "
               "not "
               "matched?  Use --skip_missing_forwarding_entries to ignore "
               "this.";

        // If we have access to the factory, use it to fix the realtime time.
        if (event_loop_factory_ != nullptr) {
          event_loop_factory_->SetRealtimeOffset(
              monotonic_clock::time_point(chrono::nanoseconds(
                  channel_data.message().monotonic_sent_time())),
              realtime_clock::time_point(chrono::nanoseconds(
                  channel_data.message().realtime_sent_time())));
        }

        channels_[channel_index]->Send(
            channel_data.message().data()->Data(),
            channel_data.message().data()->size(),
            monotonic_clock::time_point(chrono::nanoseconds(
                channel_data.message().monotonic_remote_time())),
            realtime_clock::time_point(chrono::nanoseconds(
                channel_data.message().realtime_remote_time())),
            channel_data.message().remote_queue_index());
      }
    } else {
      LOG(WARNING) << "Not sending data from before the start of the log file. "
                   << channel_timestamp.time_since_epoch().count() << " start "
                   << monotonic_start_time().time_since_epoch().count() << " "
                   << FlatbufferToJson(channel_data);
    }

    if (sorted_message_reader_.active_channel_count() > 0u) {
      timer_handler_->Setup(sorted_message_reader_.oldest_message().first);
    } else {
      // Set a timer up immediately after now to die. If we don't do this, then
      // the senders waiting on the message we just read will never get called.
      timer_handler_->Setup(monotonic_now + event_loop_factory_->send_delay() +
                            std::chrono::nanoseconds(1));
    }
  });

  if (sorted_message_reader_.active_channel_count() > 0u) {
    event_loop_->OnRun([this]() {
      timer_handler_->Setup(sorted_message_reader_.oldest_message().first);
    });
  }
}

void LogReader::Deregister() {
  // Make sure that things get destroyed in the correct order, rather than
  // relying on getting the order correct in the class definition.
  for (size_t i = 0; i < channels_.size(); ++i) {
    channels_[i].reset();
  }

  event_loop_unique_ptr_.reset();
  event_loop_ = nullptr;
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
  // If no remapping occurred and we are using the original config, then there
  // is nothing interesting to do here.
  if (remapped_channels_.empty() && replay_configuration_ == nullptr) {
    remapped_configuration_ = sorted_message_reader_.configuration();
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
  new_config_fbb.ForceDefaults(1);
  std::vector<flatbuffers::Offset<Channel>> channel_offsets;
  for (auto &pair : remapped_channels_) {
    // This is the builder that we use for creating the Channel with just the
    // new name.
    flatbuffers::FlatBufferBuilder new_name_fbb;
    new_name_fbb.ForceDefaults(1);
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

}  // namespace logger
}  // namespace aos
