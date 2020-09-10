#include "aos/events/logging/logger.h"

#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <vector>

#include "Eigen/Dense"
#include "absl/strings/escaping.h"
#include "absl/types/span.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/logging/uuid.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "flatbuffers/flatbuffers.h"
#include "third_party/gmp/gmpxx.h"

DEFINE_bool(skip_missing_forwarding_entries, false,
            "If true, drop any forwarding entries with missing data.  If "
            "false, CHECK.");

DEFINE_bool(timestamps_to_csv, false,
            "If true, write all the time synchronization information to a set "
            "of CSV files in /tmp/.  This should only be needed when debugging "
            "time synchronization.");

DEFINE_bool(skip_order_validation, false,
            "If true, ignore any out of orderness in replay");

namespace aos {
namespace logger {
namespace chrono = std::chrono;

void LogNamer::UpdateHeader(
    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
    const UUID &uuid, int parts_index) {
  header->mutable_message()->mutate_parts_index(parts_index);
  CHECK_EQ(uuid.string_view().size(),
           header->mutable_message()->mutable_parts_uuid()->size());
  std::copy(uuid.string_view().begin(), uuid.string_view().end(),
            reinterpret_cast<char *>(
                header->mutable_message()->mutable_parts_uuid()->Data()));
}

void MultiNodeLogNamer::WriteHeader(
    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
    const Node *node) {
  if (node == this->node()) {
    UpdateHeader(header, uuid_, part_number_);
    data_writer_->WriteSizedFlatbuffer(header->full_span());
  } else {
    for (std::pair<const Channel *const, DataWriter> &data_writer :
         data_writers_) {
      if (node == data_writer.second.node) {
        UpdateHeader(header, data_writer.second.uuid,
                     data_writer.second.part_number);
        data_writer.second.writer->WriteSizedFlatbuffer(header->full_span());
      }
    }
  }
}

void MultiNodeLogNamer::Rotate(
    const Node *node,
    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header) {
  if (node == this->node()) {
    ++part_number_;
    *data_writer_ = std::move(*OpenDataWriter());
    UpdateHeader(header, uuid_, part_number_);
    data_writer_->WriteSizedFlatbuffer(header->full_span());
  } else {
    for (std::pair<const Channel *const, DataWriter> &data_writer :
         data_writers_) {
      if (node == data_writer.second.node) {
        ++data_writer.second.part_number;
        data_writer.second.rotate(data_writer.first, &data_writer.second);
        UpdateHeader(header, data_writer.second.uuid,
                     data_writer.second.part_number);
        data_writer.second.writer->WriteSizedFlatbuffer(header->full_span());
      }
    }
  }
}

Logger::Logger(std::string_view base_name, EventLoop *event_loop,
               std::chrono::milliseconds polling_period)
    : Logger(std::make_unique<LocalLogNamer>(base_name, event_loop->node()),
             event_loop, polling_period) {}

Logger::Logger(std::unique_ptr<LogNamer> log_namer, EventLoop *event_loop,
               std::chrono::milliseconds polling_period)
    : event_loop_(event_loop),
      uuid_(UUID::Random()),
      log_namer_(std::move(log_namer)),
      timer_handler_(event_loop_->AddTimer([this]() { DoLogData(); })),
      polling_period_(polling_period),
      server_statistics_fetcher_(
          configuration::MultiNode(event_loop_->configuration())
              ? event_loop_->MakeFetcher<message_bridge::ServerStatistics>(
                    "/aos")
              : aos::Fetcher<message_bridge::ServerStatistics>()) {
  VLOG(1) << "Starting logger for " << FlatbufferToJson(event_loop_->node());
  int channel_index = 0;

  // Find all the nodes which are logging timestamps on our node.
  std::set<const Node *> timestamp_logger_nodes;
  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    if (!configuration::ChannelIsSendableOnNode(channel, event_loop_->node()) ||
        !channel->has_destination_nodes()) {
      continue;
    }
    for (const Connection *connection : *channel->destination_nodes()) {
      const Node *other_node = configuration::GetNode(
          event_loop_->configuration(), connection->name()->string_view());

      if (configuration::ConnectionDeliveryTimeIsLoggedOnNode(
              connection, event_loop_->node())) {
        VLOG(1) << "Timestamps are logged from "
                << FlatbufferToJson(other_node);
        timestamp_logger_nodes.insert(other_node);
      }
    }
  }

  std::map<const Channel *, const Node *> timestamp_logger_channels;

  // Now that we have all the nodes accumulated, make remote timestamp loggers
  // for them.
  for (const Node *node : timestamp_logger_nodes) {
    const Channel *channel = configuration::GetChannel(
        event_loop_->configuration(),
        absl::StrCat("/aos/remote_timestamps/", node->name()->string_view()),
        logger::MessageHeader::GetFullyQualifiedName(), event_loop_->name(),
        event_loop_->node());

    CHECK(channel != nullptr)
        << ": Remote timestamps are logged on "
        << event_loop_->node()->name()->string_view()
        << " but can't find channel /aos/remote_timestamps/"
        << node->name()->string_view();
    timestamp_logger_channels.insert(std::make_pair(channel, node));
  }

  const size_t our_node_index = configuration::GetNodeIndex(
      event_loop_->configuration(), event_loop_->node());

  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    FetcherStruct fs;
    fs.node_index = our_node_index;
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

    // Now, detect a MessageHeader timestamp logger where we should just log the
    // contents to a file directly.
    const bool log_contents = timestamp_logger_channels.find(channel) !=
                              timestamp_logger_channels.end();
    const Node *timestamp_node =
        log_contents ? timestamp_logger_channels.find(channel)->second
                     : nullptr;

    if (log_message || log_delivery_times || log_contents) {
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
      if (log_contents) {
        VLOG(1) << "Timestamp logger channel "
                << configuration::CleanedChannelToString(channel);
        fs.contents_writer =
            log_namer_->MakeForwardedTimestampWriter(channel, timestamp_node);
        fs.node_index = configuration::GetNodeIndex(
            event_loop_->configuration(), timestamp_node);
      }
      fs.channel_index = channel_index;
      fs.written = false;
      fetchers_.emplace_back(std::move(fs));
    }
    ++channel_index;
  }

  node_state_.resize(configuration::MultiNode(event_loop_->configuration())
                         ? event_loop_->configuration()->nodes()->size()
                         : 1u);

  for (const Node *node : log_namer_->nodes()) {
    const int node_index =
        configuration::GetNodeIndex(event_loop_->configuration(), node);

    node_state_[node_index].log_file_header = MakeHeader(node);
  }

  // When things start, we want to log the header, then the most recent
  // messages available on each fetcher to capture the previous state, then
  // start polling.
  event_loop_->OnRun([this]() { StartLogging(); });
}

void Logger::StartLogging() {
  // Grab data from each channel right before we declare the log file started
  // so we can capture the latest message on each channel.  This lets us have
  // non periodic messages with configuration that now get logged.
  for (FetcherStruct &f : fetchers_) {
    f.written = !f.fetcher->Fetch();
  }

  // Clear out any old timestamps in case we are re-starting logging.
  for (size_t i = 0; i < node_state_.size(); ++i) {
    SetStartTime(i, monotonic_clock::min_time, realtime_clock::min_time);
  }

  WriteHeader();

  LOG(INFO) << "Logging node as " << FlatbufferToJson(event_loop_->node())
            << " start_time " << last_synchronized_time_;

  timer_handler_->Setup(event_loop_->monotonic_now() + polling_period_,
                        polling_period_);
}

void Logger::WriteHeader() {
  if (configuration::MultiNode(event_loop_->configuration())) {
    server_statistics_fetcher_.Fetch();
  }

  aos::monotonic_clock::time_point monotonic_start_time =
      event_loop_->monotonic_now();
  aos::realtime_clock::time_point realtime_start_time =
      event_loop_->realtime_now();

  // We need to pick a point in time to declare the log file "started".  This
  // starts here.  It needs to be after everything is fetched so that the
  // fetchers are all pointed at the most recent message before the start
  // time.
  last_synchronized_time_ = monotonic_start_time;

  for (const Node *node : log_namer_->nodes()) {
    const int node_index =
        configuration::GetNodeIndex(event_loop_->configuration(), node);
    MaybeUpdateTimestamp(node, node_index, monotonic_start_time,
                         realtime_start_time);
    log_namer_->WriteHeader(&node_state_[node_index].log_file_header, node);
  }
}

void Logger::WriteMissingTimestamps() {
  if (configuration::MultiNode(event_loop_->configuration())) {
    server_statistics_fetcher_.Fetch();
  } else {
    return;
  }

  if (server_statistics_fetcher_.get() == nullptr) {
    return;
  }

  for (const Node *node : log_namer_->nodes()) {
    const int node_index =
        configuration::GetNodeIndex(event_loop_->configuration(), node);
    if (MaybeUpdateTimestamp(
            node, node_index,
            server_statistics_fetcher_.context().monotonic_event_time,
            server_statistics_fetcher_.context().realtime_event_time)) {
      log_namer_->Rotate(node, &node_state_[node_index].log_file_header);
    }
  }
}

void Logger::SetStartTime(size_t node_index,
                          aos::monotonic_clock::time_point monotonic_start_time,
                          aos::realtime_clock::time_point realtime_start_time) {
  node_state_[node_index].monotonic_start_time = monotonic_start_time;
  node_state_[node_index].realtime_start_time = realtime_start_time;
  node_state_[node_index]
      .log_file_header.mutable_message()
      ->mutate_monotonic_start_time(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              monotonic_start_time.time_since_epoch())
              .count());
  if (node_state_[node_index]
          .log_file_header.mutable_message()
          ->has_realtime_start_time()) {
    node_state_[node_index]
        .log_file_header.mutable_message()
        ->mutate_realtime_start_time(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                realtime_start_time.time_since_epoch())
                .count());
  }
}

bool Logger::MaybeUpdateTimestamp(
    const Node *node, int node_index,
    aos::monotonic_clock::time_point monotonic_start_time,
    aos::realtime_clock::time_point realtime_start_time) {
  // Bail early if there the start times are already set.
  if (node_state_[node_index].monotonic_start_time !=
      monotonic_clock::min_time) {
    return false;
  }
  if (configuration::MultiNode(event_loop_->configuration())) {
    if (event_loop_->node() == node) {
      // There are no offsets to compute for ourself, so always succeed.
      SetStartTime(node_index, monotonic_start_time, realtime_start_time);
      return true;
    } else if (server_statistics_fetcher_.get() != nullptr) {
      // We must be a remote node now.  Look for the connection and see if it is
      // connected.

      for (const message_bridge::ServerConnection *connection :
           *server_statistics_fetcher_->connections()) {
        if (connection->node()->name()->string_view() !=
            node->name()->string_view()) {
          continue;
        }

        if (connection->state() != message_bridge::State::CONNECTED) {
          VLOG(1) << node->name()->string_view()
                  << " is not connected, can't start it yet.";
          break;
        }

        if (!connection->has_monotonic_offset()) {
          VLOG(1) << "Missing monotonic offset for setting start time for node "
                  << aos::FlatbufferToJson(node);
          break;
        }

        VLOG(1) << "Updating start time for " << aos::FlatbufferToJson(node);

        // Found it and it is connected.  Compensate and go.
        monotonic_start_time +=
            std::chrono::nanoseconds(connection->monotonic_offset());

        SetStartTime(node_index, monotonic_start_time, realtime_start_time);
        return true;
      }
    }
  } else {
    SetStartTime(node_index, monotonic_start_time, realtime_start_time);
    return true;
  }
  return false;
}

aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> Logger::MakeHeader(
    const Node *node) {
  // Now write the header with this timestamp in it.
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  // TODO(austin): Compress this much more efficiently.  There are a bunch of
  // duplicated schemas.
  flatbuffers::Offset<aos::Configuration> configuration_offset =
      CopyFlatBuffer(event_loop_->configuration(), &fbb);

  flatbuffers::Offset<flatbuffers::String> name_offset =
      fbb.CreateString(network::GetHostname());

  flatbuffers::Offset<flatbuffers::String> logger_uuid_offset =
      fbb.CreateString(uuid_.string_view());

  flatbuffers::Offset<flatbuffers::String> parts_uuid_offset =
      fbb.CreateString("00000000-0000-4000-8000-000000000000");

  flatbuffers::Offset<Node> node_offset;

  if (configuration::MultiNode(event_loop_->configuration())) {
    node_offset = CopyFlatBuffer(node, &fbb);
  }

  aos::logger::LogFileHeader::Builder log_file_header_builder(fbb);

  log_file_header_builder.add_name(name_offset);

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
          monotonic_clock::min_time.time_since_epoch())
          .count());
  if (node == event_loop_->node()) {
    log_file_header_builder.add_realtime_start_time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            realtime_clock::min_time.time_since_epoch())
            .count());
  }

  log_file_header_builder.add_logger_uuid(logger_uuid_offset);

  log_file_header_builder.add_parts_uuid(parts_uuid_offset);
  log_file_header_builder.add_parts_index(0);

  fbb.FinishSizePrefixed(log_file_header_builder.Finish());
  return fbb.Release();
}

void Logger::Rotate() {
  for (const Node *node : log_namer_->nodes()) {
    const int node_index =
        configuration::GetNodeIndex(event_loop_->configuration(), node);
    log_namer_->Rotate(node, &node_state_[node_index].log_file_header);
  }
}

void Logger::LogUntil(monotonic_clock::time_point t) {
  WriteMissingTimestamps();

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
      if (f.fetcher->context().monotonic_event_time < t) {
        if (f.writer != nullptr) {
          // Write!
          flatbuffers::FlatBufferBuilder fbb(f.fetcher->context().size +
                                             max_header_size_);
          fbb.ForceDefaults(true);

          fbb.FinishSizePrefixed(PackMessage(&fbb, f.fetcher->context(),
                                             f.channel_index, f.log_type));

          VLOG(2) << "Writing data as node "
                  << FlatbufferToJson(event_loop_->node()) << " for channel "
                  << configuration::CleanedChannelToString(f.fetcher->channel())
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
                  << configuration::CleanedChannelToString(f.fetcher->channel())
                  << " to " << f.timestamp_writer->filename() << " timestamp "
                  << FlatbufferToJson(
                         flatbuffers::GetSizePrefixedRoot<MessageHeader>(
                             fbb.GetBufferPointer()));

          f.timestamp_writer->QueueSizedFlatbuffer(&fbb);
        }

        if (f.contents_writer != nullptr) {
          // And now handle the special message contents channel.  Copy the
          // message into a FlatBufferBuilder and save it to disk.
          // TODO(austin): We can be more efficient here when we start to
          // care...
          flatbuffers::FlatBufferBuilder fbb;
          fbb.ForceDefaults(true);

          const MessageHeader *msg =
              flatbuffers::GetRoot<MessageHeader>(f.fetcher->context().data);

          logger::MessageHeader::Builder message_header_builder(fbb);

          // Note: this must match the same order as MessageBridgeServer and
          // PackMessage.  We want identical headers to have identical
          // on-the-wire formats to make comparing them easier.
          message_header_builder.add_channel_index(msg->channel_index());

          message_header_builder.add_queue_index(msg->queue_index());
          message_header_builder.add_monotonic_sent_time(
              msg->monotonic_sent_time());
          message_header_builder.add_realtime_sent_time(
              msg->realtime_sent_time());

          message_header_builder.add_monotonic_remote_time(
              msg->monotonic_remote_time());
          message_header_builder.add_realtime_remote_time(
              msg->realtime_remote_time());
          message_header_builder.add_remote_queue_index(
              msg->remote_queue_index());

          fbb.FinishSizePrefixed(message_header_builder.Finish());

          f.contents_writer->QueueSizedFlatbuffer(&fbb);
        }

        f.written = true;
      } else {
        break;
      }
    }
  }
  last_synchronized_time_ = t;
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
    LogUntil(
        std::min(last_synchronized_time_ + polling_period_, monotonic_now));

    // If we missed cycles, we could be pretty far behind.  Spin until we are
    // caught up.
  } while (last_synchronized_time_ + polling_period_ < monotonic_now);
}

std::vector<std::vector<std::string>> SortParts(
    const std::vector<std::string> &parts) {
  // Start by grouping all parts by UUID, and extracting the part index.
  std::map<std::string, std::vector<std::pair<std::string, int>>> parts_list;

  // Sort part files without UUIDs and part indexes as well.  Extract everything
  // useful from the log in the first pass, then sort later.
  struct LogPart {
    std::string filename;
    monotonic_clock::time_point start_time;
    monotonic_clock::time_point first_message_time;
  };

  std::vector<LogPart> old_parts;

  for (const std::string &part : parts) {
    FlatbufferVector<LogFileHeader> log_header = ReadHeader(part);

    // Looks like an old log.  No UUID, index, and also single node.  We have
    // little to no multi-node log files in the wild without part UUIDs and
    // indexes which we care much about.
    if (!log_header.message().has_parts_uuid() &&
        !log_header.message().has_parts_index() &&
        !log_header.message().has_node()) {
      LogPart log_part;
      log_part.filename = part;
      log_part.start_time = monotonic_clock::time_point(
          chrono::nanoseconds(log_header.message().monotonic_start_time()));
      FlatbufferVector<MessageHeader> first_message = ReadNthMessage(part, 0);
      log_part.first_message_time = monotonic_clock::time_point(
          chrono::nanoseconds(first_message.message().monotonic_sent_time()));
      old_parts.emplace_back(std::move(log_part));
      continue;
    }

    CHECK(log_header.message().has_parts_uuid());
    CHECK(log_header.message().has_parts_index());

    const std::string parts_uuid = log_header.message().parts_uuid()->str();
    auto it = parts_list.find(parts_uuid);
    if (it == parts_list.end()) {
      it = parts_list
               .insert(std::make_pair(
                   parts_uuid, std::vector<std::pair<std::string, int>>{}))
               .first;
    }
    it->second.emplace_back(
        std::make_pair(part, log_header.message().parts_index()));
  }

  CHECK_NE(old_parts.empty(), parts_list.empty())
      << ": Can't have a mix of old and new parts.";

  if (!old_parts.empty()) {
    // Confirm they all have the same start time.  Old loggers always used the
    // same start time.
    for (const LogPart &p : old_parts) {
      CHECK_EQ(old_parts[0].start_time, p.start_time);
    }
    // Sort by the oldest message in each file.
    std::sort(old_parts.begin(), old_parts.end(),
              [](const LogPart &a, const LogPart &b) {
                return a.first_message_time < b.first_message_time;
              });

    // Produce the final form.
    std::vector<std::string> sorted_old_parts;
    sorted_old_parts.reserve(old_parts.size());
    for (LogPart &p : old_parts) {
      sorted_old_parts.emplace_back(std::move(p.filename));
    }
    return std::vector<std::vector<std::string>>{std::move(sorted_old_parts)};
  }

  // Now, sort them and produce the final vector form.
  std::vector<std::vector<std::string>> result;
  result.reserve(parts_list.size());
  for (auto &part : parts_list) {
    std::sort(part.second.begin(), part.second.end(),
              [](const std::pair<std::string, int> &a,
                 const std::pair<std::string, int> &b) {
                return a.second < b.second;
              });
    std::vector<std::string> result_line;
    result_line.reserve(part.second.size());
    for (std::pair<std::string, int> &p : part.second) {
      result_line.emplace_back(std::move(p.first));
    }
    result.emplace_back(std::move(result_line));
  }
  return result;
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
        << ": Log file and replay config need to both be multi or single "
           "node.";
  }

  if (!configuration::MultiNode(configuration())) {
    states_.emplace_back(
        std::make_unique<State>(std::make_unique<ChannelMerger>(filenames)));
  } else {
    if (replay_configuration) {
      CHECK_EQ(logged_configuration()->nodes()->size(),
               replay_configuration->nodes()->size())
          << ": Log file and replay config need to have matching nodes "
             "lists.";
      for (const Node *node : *logged_configuration()->nodes()) {
        if (configuration::GetNode(replay_configuration, node) == nullptr) {
          LOG(FATAL) << "Found node " << FlatbufferToJson(node)
                     << " in logged config that is not present in the replay "
                        "config.";
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
  // Zero out some buffers. It's easy to do use-after-frees on these, so make
  // it more obvious.
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
  // Because the Node pointer will only be valid if it actually points to
  // memory owned by remapped_configuration_, we need to wait for the
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

  return state->monotonic_start_time();
}

realtime_clock::time_point LogReader::realtime_start_time(const Node *node) {
  State *state =
      states_[configuration::GetNodeIndex(configuration(), node)].get();
  CHECK(state != nullptr) << ": Unknown node " << FlatbufferToJson(node);

  return state->realtime_start_time();
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
    states_[node_index] =
        std::make_unique<State>(std::make_unique<ChannelMerger>(filenames_));
    State *state = states_[node_index].get();

    Register(state->SetNodeEventLoopFactory(
        event_loop_factory_->GetNodeEventLoopFactory(node)));
  }
  if (live_nodes_ == 0) {
    LOG(FATAL)
        << "Don't have logs from any of the nodes in the replay config--are "
           "you sure that the replay config matches the original config?";
  }

  // We need to now seed our per-node time offsets and get everything set up
  // to run.
  const size_t num_nodes = nodes_count();

  // It is easiest to solve for per node offsets with a matrix rather than
  // trying to solve the equations by hand.  So let's get after it.
  //
  // Now, build up the map matrix.
  //
  // offset_matrix_ = (map_matrix_ + slope_matrix_) * [ta; tb; tc]
  map_matrix_ = Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic>::Zero(
      filters_.size() + 1, num_nodes);
  slope_matrix_ =
      Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic>::Zero(
          filters_.size() + 1, num_nodes);

  offset_matrix_ =
      Eigen::Matrix<mpq_class, Eigen::Dynamic, 1>::Zero(filters_.size() + 1);
  valid_matrix_ =
      Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(filters_.size() + 1);
  last_valid_matrix_ =
      Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(filters_.size() + 1);

  time_offset_matrix_ = Eigen::VectorXd::Zero(num_nodes);
  time_slope_matrix_ = Eigen::VectorXd::Zero(num_nodes);

  // All times should average out to the distributed clock.
  for (int i = 0; i < map_matrix_.cols(); ++i) {
    // 1/num_nodes.
    map_matrix_(0, i) = mpq_class(1, num_nodes);
  }
  valid_matrix_(0) = true;

  {
    // Now, add the a - b -> sample elements.
    size_t i = 1;
    for (std::pair<const std::tuple<const Node *, const Node *>,
                   std::tuple<message_bridge::NoncausalOffsetEstimator>>
             &filter : filters_) {
      const Node *const node_a = std::get<0>(filter.first);
      const Node *const node_b = std::get<1>(filter.first);

      const size_t node_a_index =
          configuration::GetNodeIndex(configuration(), node_a);
      const size_t node_b_index =
          configuration::GetNodeIndex(configuration(), node_b);

      // -a
      map_matrix_(i, node_a_index) = mpq_class(-1);
      // +b
      map_matrix_(i, node_b_index) = mpq_class(1);

      // -> sample
      std::get<0>(filter.second)
          .set_slope_pointer(&slope_matrix_(i, node_a_index));
      std::get<0>(filter.second).set_offset_pointer(&offset_matrix_(i, 0));

      valid_matrix_(i) = false;
      std::get<0>(filter.second).set_valid_pointer(&valid_matrix_(i));

      ++i;
    }
  }

  for (std::unique_ptr<State> &state : states_) {
    state->SeedSortedMessages();
  }

  // Rank of the map matrix tells you if all the nodes are in communication
  // with each other, which tells you if the offsets are observable.
  const size_t connected_nodes =
      Eigen::FullPivLU<
          Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic>>(map_matrix_)
          .rank();

  // We don't need to support isolated nodes until someone has a real use
  // case.
  CHECK_EQ(connected_nodes, num_nodes)
      << ": There is a node which isn't communicating with the rest.";

  // And solve.
  UpdateOffsets();

  // We want to start the log file at the last start time of the log files
  // from all the nodes.  Compute how long each node's simulation needs to run
  // to move time to this point.
  distributed_clock::time_point start_time = distributed_clock::min_time;

  // TODO(austin): We want an "OnStart" callback for each node rather than
  // running until the last node.

  for (std::unique_ptr<State> &state : states_) {
    VLOG(1) << "Start time is " << state->monotonic_start_time() << " for node "
            << MaybeNodeName(state->event_loop()->node()) << "now "
            << state->monotonic_now();
    // And start computing the start time on the distributed clock now that
    // that works.
    start_time = std::max(
        start_time, state->ToDistributedClock(state->monotonic_start_time()));
  }

  CHECK_GE(start_time, distributed_clock::epoch())
      << ": Hmm, we have a node starting before the start of time.  Offset "
         "everything.";

  // Forwarding is tracked per channel.  If it is enabled, we want to turn it
  // off.  Otherwise messages replayed will get forwarded across to the other
  // nodes, and also replayed on the other nodes.  This may not satisfy all
  // our users, but it'll start the discussion.
  if (configuration::MultiNode(event_loop_factory_->configuration())) {
    for (size_t i = 0; i < logged_configuration()->channels()->size(); ++i) {
      const Channel *channel = logged_configuration()->channels()->Get(i);
      const Node *node = configuration::GetNode(
          configuration(), channel->source_node()->string_view());

      State *state =
          states_[configuration::GetNodeIndex(configuration(), node)].get();

      const Channel *remapped_channel =
          RemapChannel(state->event_loop(), channel);

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
  VLOG(1) << "Running until " << start_time << " in Register";
  event_loop_factory_->RunFor(start_time.time_since_epoch());
  VLOG(1) << "At start time";
  // Now that we are running for real, missing data means that the log file is
  // corrupted or went wrong.
  ignore_missing_data_ = false;

  for (std::unique_ptr<State> &state : states_) {
    // Make the RT clock be correct before handing it to the user.
    if (state->realtime_start_time() != realtime_clock::min_time) {
      state->SetRealtimeOffset(state->monotonic_start_time(),
                               state->realtime_start_time());
    }
    VLOG(1) << "Start time is " << state->monotonic_start_time() << " for node "
            << MaybeNodeName(state->event_loop()->node()) << "now "
            << state->monotonic_now();
  }

  if (FLAGS_timestamps_to_csv) {
    for (std::pair<const std::tuple<const Node *, const Node *>,
                   std::tuple<message_bridge::NoncausalOffsetEstimator>>
             &filter : filters_) {
      const Node *const node_a = std::get<0>(filter.first);
      const Node *const node_b = std::get<1>(filter.first);

      std::get<0>(filter.second)
          .SetFirstFwdTime(event_loop_factory_->GetNodeEventLoopFactory(node_a)
                               ->monotonic_now());
      std::get<0>(filter.second)
          .SetFirstRevTime(event_loop_factory_->GetNodeEventLoopFactory(node_b)
                               ->monotonic_now());
    }
  }
}

void LogReader::UpdateOffsets() {
  VLOG(2) << "Samples are " << offset_matrix_;
  VLOG(2) << "Map is " << (map_matrix_ + slope_matrix_);
  std::tie(time_slope_matrix_, time_offset_matrix_) = SolveOffsets();
  Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[",
                           "]");
  VLOG(1) << "First slope " << time_slope_matrix_.transpose().format(HeavyFmt)
          << " offset " << time_offset_matrix_.transpose().format(HeavyFmt);

  size_t node_index = 0;
  for (std::unique_ptr<State> &state : states_) {
    state->SetDistributedOffset(offset(node_index), slope(node_index));
    VLOG(1) << "Offset for node " << node_index << " "
            << MaybeNodeName(state->event_loop()->node()) << "is "
            << aos::distributed_clock::time_point(offset(node_index))
            << " slope " << std::setprecision(9) << std::fixed
            << slope(node_index);
    ++node_index;
  }

  if (VLOG_IS_ON(1)) {
    LogFit("Offset is");
  }
}

void LogReader::LogFit(std::string_view prefix) {
  for (std::unique_ptr<State> &state : states_) {
    VLOG(1) << MaybeNodeName(state->event_loop()->node()) << " now "
            << state->monotonic_now() << " distributed "
            << event_loop_factory_->distributed_now();
  }

  for (std::pair<const std::tuple<const Node *, const Node *>,
                 std::tuple<message_bridge::NoncausalOffsetEstimator>> &filter :
       filters_) {
    message_bridge::NoncausalOffsetEstimator *estimator =
        &std::get<0>(filter.second);

    if (estimator->a_timestamps().size() == 0 &&
        estimator->b_timestamps().size() == 0) {
      continue;
    }

    if (VLOG_IS_ON(1)) {
      estimator->LogFit(prefix);
    }

    const Node *const node_a = std::get<0>(filter.first);
    const Node *const node_b = std::get<1>(filter.first);

    const size_t node_a_index =
        configuration::GetNodeIndex(configuration(), node_a);
    const size_t node_b_index =
        configuration::GetNodeIndex(configuration(), node_b);

    const double recovered_slope =
        slope(node_b_index) / slope(node_a_index) - 1.0;
    const int64_t recovered_offset =
        offset(node_b_index).count() - offset(node_a_index).count() *
                                           slope(node_b_index) /
                                           slope(node_a_index);

    VLOG(1) << "Recovered slope " << std::setprecision(20) << recovered_slope
            << " (error " << recovered_slope - estimator->fit().slope() << ") "
            << " offset " << std::setprecision(20) << recovered_offset
            << " (error "
            << recovered_offset - estimator->fit().offset().count() << ")";

    const aos::distributed_clock::time_point a0 =
        states_[node_a_index]->ToDistributedClock(
            std::get<0>(estimator->a_timestamps()[0]));
    const aos::distributed_clock::time_point a1 =
        states_[node_a_index]->ToDistributedClock(
            std::get<0>(estimator->a_timestamps()[1]));

    VLOG(1) << node_a->name()->string_view() << " timestamps()[0] = "
            << std::get<0>(estimator->a_timestamps()[0]) << " -> " << a0
            << " distributed -> " << node_b->name()->string_view() << " "
            << states_[node_b_index]->FromDistributedClock(a0) << " should be "
            << aos::monotonic_clock::time_point(
                   std::chrono::nanoseconds(static_cast<int64_t>(
                       std::get<0>(estimator->a_timestamps()[0])
                           .time_since_epoch()
                           .count() *
                       (1.0 + estimator->fit().slope()))) +
                   estimator->fit().offset())
            << ((a0 <= event_loop_factory_->distributed_now())
                    ? ""
                    : " After now, investigate");
    VLOG(1) << node_a->name()->string_view() << " timestamps()[1] = "
            << std::get<0>(estimator->a_timestamps()[1]) << " -> " << a1
            << " distributed -> " << node_b->name()->string_view() << " "
            << states_[node_b_index]->FromDistributedClock(a1) << " should be "
            << aos::monotonic_clock::time_point(
                   std::chrono::nanoseconds(static_cast<int64_t>(
                       std::get<0>(estimator->a_timestamps()[1])
                           .time_since_epoch()
                           .count() *
                       (1.0 + estimator->fit().slope()))) +
                   estimator->fit().offset())
            << ((event_loop_factory_->distributed_now() <= a1)
                    ? ""
                    : " Before now, investigate");

    const aos::distributed_clock::time_point b0 =
        states_[node_b_index]->ToDistributedClock(
            std::get<0>(estimator->b_timestamps()[0]));
    const aos::distributed_clock::time_point b1 =
        states_[node_b_index]->ToDistributedClock(
            std::get<0>(estimator->b_timestamps()[1]));

    VLOG(1) << node_b->name()->string_view() << " timestamps()[0] = "
            << std::get<0>(estimator->b_timestamps()[0]) << " -> " << b0
            << " distributed -> " << node_a->name()->string_view() << " "
            << states_[node_a_index]->FromDistributedClock(b0)
            << ((b0 <= event_loop_factory_->distributed_now())
                    ? ""
                    : " After now, investigate");
    VLOG(1) << node_b->name()->string_view() << " timestamps()[1] = "
            << std::get<0>(estimator->b_timestamps()[1]) << " -> " << b1
            << " distributed -> " << node_a->name()->string_view() << " "
            << states_[node_a_index]->FromDistributedClock(b1)
            << ((event_loop_factory_->distributed_now() <= b1)
                    ? ""
                    : " Before now, investigate");
  }
}

message_bridge::NoncausalOffsetEstimator *LogReader::GetFilter(
    const Node *node_a, const Node *node_b) {
  CHECK_NE(node_a, node_b);
  CHECK_EQ(configuration::GetNode(configuration(), node_a), node_a);
  CHECK_EQ(configuration::GetNode(configuration(), node_b), node_b);

  if (node_a > node_b) {
    return GetFilter(node_b, node_a);
  }

  auto tuple = std::make_tuple(node_a, node_b);

  auto it = filters_.find(tuple);

  if (it == filters_.end()) {
    auto &x =
        filters_
            .insert(std::make_pair(
                tuple, std::make_tuple(message_bridge::NoncausalOffsetEstimator(
                           node_a, node_b))))
            .first->second;
    if (FLAGS_timestamps_to_csv) {
      std::get<0>(x).SetFwdCsvFileName(absl::StrCat(
          "/tmp/timestamp_noncausal_", node_a->name()->string_view(), "_",
          node_b->name()->string_view()));
      std::get<0>(x).SetRevCsvFileName(absl::StrCat(
          "/tmp/timestamp_noncausal_", node_b->name()->string_view(), "_",
          node_a->name()->string_view()));
    }

    return &std::get<0>(x);
  } else {
    return &std::get<0>(it->second);
  }
}


void LogReader::Register(EventLoop *event_loop) {
  State *state =
      states_[configuration::GetNodeIndex(configuration(), event_loop->node())]
          .get();

  state->set_event_loop(event_loop);

  // We don't run timing reports when trying to print out logged data, because
  // otherwise we would end up printing out the timing reports themselves...
  // This is only really relevant when we are replaying into a simulation.
  event_loop->SkipTimingReport();
  event_loop->SkipAosLog();

  const bool has_data = state->SetNode();

  state->SetChannelCount(logged_configuration()->channels()->size());

  for (size_t i = 0; i < logged_configuration()->channels()->size(); ++i) {
    const Channel *channel =
        RemapChannel(event_loop, logged_configuration()->channels()->Get(i));

    NodeEventLoopFactory *channel_target_event_loop_factory = nullptr;
    message_bridge::NoncausalOffsetEstimator *filter = nullptr;

    if (!configuration::ChannelIsSendableOnNode(channel, event_loop->node()) &&
        configuration::ChannelIsReadableOnNode(channel, event_loop->node())) {
      const Node *target_node = configuration::GetNode(
          event_loop->configuration(), channel->source_node()->string_view());
      filter = GetFilter(event_loop->node(), target_node);

      if (event_loop_factory_ != nullptr) {
        channel_target_event_loop_factory =
            event_loop_factory_->GetNodeEventLoopFactory(target_node);
      }
    }

    state->SetChannel(i, event_loop->MakeRawSender(channel), filter,
                      channel_target_event_loop_factory);
  }

  // If we didn't find any log files with data in them, we won't ever get a
  // callback or be live.  So skip the rest of the setup.
  if (!has_data) {
    return;
  }

  state->set_timer_handler(event_loop->AddTimer([this, state]() {
    VLOG(1) << "Starting sending " << MaybeNodeName(state->event_loop()->node())
            << "at " << state->event_loop()->context().monotonic_event_time
            << " now " << state->monotonic_now();
    if (state->OldestMessageTime() == monotonic_clock::max_time) {
      --live_nodes_;
      VLOG(1) << MaybeNodeName(state->event_loop()->node()) << "Node down!";
      if (live_nodes_ == 0) {
        event_loop_factory_->Exit();
      }
      return;
    }
    TimestampMerger::DeliveryTimestamp channel_timestamp;
    int channel_index;
    FlatbufferVector<MessageHeader> channel_data =
        FlatbufferVector<MessageHeader>::Empty();

    if (VLOG_IS_ON(1)) {
      LogFit("Offset was");
    }

    bool update_time;
    std::tie(channel_timestamp, channel_index, channel_data) =
        state->PopOldest(&update_time);

    const monotonic_clock::time_point monotonic_now =
        state->event_loop()->context().monotonic_event_time;
    if (!FLAGS_skip_order_validation) {
      CHECK(monotonic_now == channel_timestamp.monotonic_event_time)
          << ": " << FlatbufferToJson(state->event_loop()->node()) << " Now "
          << monotonic_now << " trying to send "
          << channel_timestamp.monotonic_event_time << " failure "
          << state->DebugString();
    } else if (monotonic_now != channel_timestamp.monotonic_event_time) {
      LOG(WARNING) << "Check failed: monotonic_now == "
                      "channel_timestamp.monotonic_event_time) ("
                   << monotonic_now << " vs. "
                   << channel_timestamp.monotonic_event_time
                   << "): " << FlatbufferToJson(state->event_loop()->node())
                   << " Now " << monotonic_now << " trying to send "
                   << channel_timestamp.monotonic_event_time << " failure "
                   << state->DebugString();
    }

    if (channel_timestamp.monotonic_event_time >
            state->monotonic_start_time() ||
        event_loop_factory_ != nullptr) {
      if ((!ignore_missing_data_ && !FLAGS_skip_missing_forwarding_entries &&
           !state->at_end()) ||
          channel_data.message().data() != nullptr) {
        CHECK(channel_data.message().data() != nullptr)
            << ": Got a message without data.  Forwarding entry which was "
               "not matched?  Use --skip_missing_forwarding_entries to "
               "ignore "
               "this.";

        if (update_time) {
          // Confirm that the message was sent on the sending node before the
          // destination node (this node).  As a proxy, do this by making sure
          // that time on the source node is past when the message was sent.
          if (!FLAGS_skip_order_validation) {
            CHECK_LT(channel_timestamp.monotonic_remote_time,
                     state->monotonic_remote_now(channel_index))
                << state->event_loop()->node()->name()->string_view() << " to "
                << state->remote_node(channel_index)->name()->string_view()
                << " " << state->DebugString();
          } else if (channel_timestamp.monotonic_remote_time >=
                     state->monotonic_remote_now(channel_index)) {
            LOG(WARNING)
                << "Check failed: channel_timestamp.monotonic_remote_time < "
                   "state->monotonic_remote_now(channel_index) ("
                << channel_timestamp.monotonic_remote_time << " vs. "
                << state->monotonic_remote_now(channel_index) << ") "
                << state->event_loop()->node()->name()->string_view() << " to "
                << state->remote_node(channel_index)->name()->string_view()
                << " currently " << channel_timestamp.monotonic_event_time
                << " ("
                << state->ToDistributedClock(
                       channel_timestamp.monotonic_event_time)
                << ") remote event time "
                << channel_timestamp.monotonic_remote_time << " ("
                << state->RemoteToDistributedClock(
                       channel_index, channel_timestamp.monotonic_remote_time)
                << ") " << state->DebugString();
          }

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
            for (int i = 1; i < time_offset_matrix_.rows(); ++i) {
              fprintf(offset_fp_, ", %.9f",
                      time_offset_matrix_(i, 0) +
                          time_slope_matrix_(i, 0) *
                              chrono::duration<double>(
                                  event_loop_factory_->distributed_now()
                                      .time_since_epoch())
                                  .count());
            }
            fprintf(offset_fp_, "\n");
          }
        }

        // If we have access to the factory, use it to fix the realtime time.
        state->SetRealtimeOffset(channel_timestamp.monotonic_event_time,
                                 channel_timestamp.realtime_event_time);

        VLOG(1) << MaybeNodeName(state->event_loop()->node()) << "Sending "
                << channel_timestamp.monotonic_event_time;
        // TODO(austin): std::move channel_data in and make that efficient in
        // simulation.
        state->Send(channel_index, channel_data.message().data()->Data(),
                    channel_data.message().data()->size(),
                    channel_timestamp.monotonic_remote_time,
                    channel_timestamp.realtime_remote_time,
                    channel_timestamp.remote_queue_index);
      } else if (state->at_end() && !ignore_missing_data_) {
        // We are at the end of the log file and found missing data.  Finish
        // reading the rest of the log file and call it quits.  We don't want
        // to replay partial data.
        while (state->OldestMessageTime() != monotonic_clock::max_time) {
          bool update_time_dummy;
          state->PopOldest(&update_time_dummy);
        }
      } else {
        CHECK(channel_data.message().data() == nullptr) << ": Nullptr";
      }
    } else {
      LOG(WARNING)
          << "Not sending data from before the start of the log file. "
          << channel_timestamp.monotonic_event_time.time_since_epoch().count()
          << " start " << monotonic_start_time().time_since_epoch().count()
          << " " << FlatbufferToJson(channel_data);
    }

    const monotonic_clock::time_point next_time = state->OldestMessageTime();
    if (next_time != monotonic_clock::max_time) {
      VLOG(1) << "Scheduling " << MaybeNodeName(state->event_loop()->node())
              << "wakeup for " << next_time << "("
              << state->ToDistributedClock(next_time)
              << " distributed), now is " << state->monotonic_now();
      state->Setup(next_time);
    } else {
      VLOG(1) << MaybeNodeName(state->event_loop()->node())
              << "No next message, scheduling shutdown";
      // Set a timer up immediately after now to die. If we don't do this,
      // then the senders waiting on the message we just read will never get
      // called.
      if (event_loop_factory_ != nullptr) {
        state->Setup(monotonic_now + event_loop_factory_->send_delay() +
                     std::chrono::nanoseconds(1));
      }
    }

    // Once we make this call, the current time changes.  So do everything
    // which involves time before changing it.  That especially includes
    // sending the message.
    if (update_time) {
      VLOG(1) << MaybeNodeName(state->event_loop()->node())
              << "updating offsets";

      std::vector<aos::monotonic_clock::time_point> before_times;
      before_times.resize(states_.size());
      std::transform(states_.begin(), states_.end(), before_times.begin(),
                     [](const std::unique_ptr<State> &state) {
                       return state->monotonic_now();
                     });

      for (size_t i = 0; i < states_.size(); ++i) {
        VLOG(1) << MaybeNodeName(
                       states_[i]->event_loop()->node())
                << "before " << states_[i]->monotonic_now();
      }

      UpdateOffsets();
      VLOG(1) << MaybeNodeName(state->event_loop()->node()) << "Now is now "
              << state->monotonic_now();

      for (size_t i = 0; i < states_.size(); ++i) {
        VLOG(1) << MaybeNodeName(
                       states_[i]->event_loop()->node())
                << "after " << states_[i]->monotonic_now();
      }

      // TODO(austin): We should be perfect.
      const std::chrono::nanoseconds kTolerance{3};
      if (!FLAGS_skip_order_validation) {
        CHECK_GE(next_time, state->monotonic_now())
            << ": Time skipped the next event.";

        for (size_t i = 0; i < states_.size(); ++i) {
          CHECK_GE(states_[i]->monotonic_now(), before_times[i] - kTolerance)
              << ": Time changed too much on node "
              << MaybeNodeName(states_[i]->event_loop()->node());
          CHECK_LE(states_[i]->monotonic_now(), before_times[i] + kTolerance)
              << ": Time changed too much on node "
              << states_[i]->event_loop()->node()->name()->string_view();
        }
      } else {
        if (next_time < state->monotonic_now()) {
          LOG(WARNING) << "Check failed: next_time >= "
                          "state->monotonic_now() ("
                       << next_time << " vs. " << state->monotonic_now()
                       << "): Time skipped the next event.";
        }
        for (size_t i = 0; i < states_.size(); ++i) {
          if (states_[i]->monotonic_now() >= before_times[i] - kTolerance) {
            LOG(WARNING) << "Check failed: "
                            "states_[i]->monotonic_now() "
                            ">= before_times[i] - kTolerance ("
                         << states_[i]->monotonic_now() << " vs. "
                         << before_times[i] - kTolerance
                         << ") : Time changed too much on node "
                         << MaybeNodeName(states_[i]->event_loop()->node());
          }
          if (states_[i]->monotonic_now() <= before_times[i] + kTolerance) {
            LOG(WARNING) << "Check failed: "
                            "states_[i]->monotonic_now() "
                            "<= before_times[i] + kTolerance ("
                         << states_[i]->monotonic_now() << " vs. "
                         << before_times[i] - kTolerance
                         << ") : Time changed too much on node "
                         << MaybeNodeName(states_[i]->event_loop()->node());
          }
        }
      }
    }

    VLOG(1) << MaybeNodeName(state->event_loop()->node()) << "Done sending at "
            << state->event_loop()->context().monotonic_event_time << " now "
            << state->monotonic_now();
  }));

  ++live_nodes_;

  if (state->OldestMessageTime() != monotonic_clock::max_time) {
    event_loop->OnRun([state]() { state->Setup(state->OldestMessageTime()); });
  }
}

void LogReader::Deregister() {
  // Make sure that things get destroyed in the correct order, rather than
  // relying on getting the order correct in the class definition.
  for (std::unique_ptr<State> &state : states_) {
    state->Deregister();
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
      CHECK(!state->event_loop())
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
    // Retrieve the channel that we want to copy, confirming that it is
    // actually present in base_config.
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

LogReader::State::State(std::unique_ptr<ChannelMerger> channel_merger)
    : channel_merger_(std::move(channel_merger)) {}

EventLoop *LogReader::State::SetNodeEventLoopFactory(
    NodeEventLoopFactory *node_event_loop_factory) {
  node_event_loop_factory_ = node_event_loop_factory;
  event_loop_unique_ptr_ =
      node_event_loop_factory_->MakeEventLoop("log_reader");
  return event_loop_unique_ptr_.get();
}

void LogReader::State::SetChannelCount(size_t count) {
  channels_.resize(count);
  filters_.resize(count);
  channel_target_event_loop_factory_.resize(count);
}

void LogReader::State::SetChannel(
    size_t channel, std::unique_ptr<RawSender> sender,
    message_bridge::NoncausalOffsetEstimator *filter,
    NodeEventLoopFactory *channel_target_event_loop_factory) {
  channels_[channel] = std::move(sender);
  filters_[channel] = filter;
  channel_target_event_loop_factory_[channel] =
      channel_target_event_loop_factory;
}

std::tuple<TimestampMerger::DeliveryTimestamp, int,
           FlatbufferVector<MessageHeader>>
LogReader::State::PopOldest(bool *update_time) {
  CHECK_GT(sorted_messages_.size(), 0u);

  std::tuple<TimestampMerger::DeliveryTimestamp, int,
             FlatbufferVector<MessageHeader>,
             message_bridge::NoncausalOffsetEstimator *>
      result = std::move(sorted_messages_.front());
  VLOG(2) << MaybeNodeName(event_loop_->node()) << "PopOldest Popping "
          << std::get<0>(result).monotonic_event_time;
  sorted_messages_.pop_front();
  SeedSortedMessages();

  if (std::get<3>(result) != nullptr) {
    *update_time = std::get<3>(result)->Pop(
        event_loop_->node(), std::get<0>(result).monotonic_event_time);
  } else {
    *update_time = false;
  }
  return std::make_tuple(std::get<0>(result), std::get<1>(result),
                         std::move(std::get<2>(result)));
}

monotonic_clock::time_point LogReader::State::OldestMessageTime() const {
  if (sorted_messages_.size() > 0) {
    VLOG(2) << MaybeNodeName(event_loop_->node()) << "oldest message at "
            << std::get<0>(sorted_messages_.front()).monotonic_event_time;
    return std::get<0>(sorted_messages_.front()).monotonic_event_time;
  }

  return channel_merger_->OldestMessageTime();
}

void LogReader::State::SeedSortedMessages() {
  const aos::monotonic_clock::time_point end_queue_time =
      (sorted_messages_.size() > 0
           ? std::get<0>(sorted_messages_.front()).monotonic_event_time
           : channel_merger_->monotonic_start_time()) +
      std::chrono::seconds(2);

  while (true) {
    if (channel_merger_->OldestMessageTime() == monotonic_clock::max_time) {
      return;
    }
    if (sorted_messages_.size() > 0) {
      // Stop placing sorted messages on the list once we have 2 seconds
      // queued up (but queue at least until the log starts.
      if (end_queue_time <
          std::get<0>(sorted_messages_.back()).monotonic_event_time) {
        return;
      }
    }

    TimestampMerger::DeliveryTimestamp channel_timestamp;
    int channel_index;
    FlatbufferVector<MessageHeader> channel_data =
        FlatbufferVector<MessageHeader>::Empty();

    message_bridge::NoncausalOffsetEstimator *filter = nullptr;

    std::tie(channel_timestamp, channel_index, channel_data) =
        channel_merger_->PopOldest();

    // Skip any messages without forwarding information.
    if (channel_timestamp.monotonic_remote_time != monotonic_clock::min_time) {
      // Got a forwarding timestamp!
      filter = filters_[channel_index];

      CHECK(filter != nullptr);

      // Call the correct method depending on if we are the forward or
      // reverse direction here.
      filter->Sample(event_loop_->node(),
                     channel_timestamp.monotonic_event_time,
                     channel_timestamp.monotonic_remote_time);
    }
    sorted_messages_.emplace_back(channel_timestamp, channel_index,
                                  std::move(channel_data), filter);
  }
}

void LogReader::State::Deregister() {
  for (size_t i = 0; i < channels_.size(); ++i) {
    channels_[i].reset();
  }
  event_loop_unique_ptr_.reset();
  event_loop_ = nullptr;
  timer_handler_ = nullptr;
  node_event_loop_factory_ = nullptr;
}

}  // namespace logger
}  // namespace aos
