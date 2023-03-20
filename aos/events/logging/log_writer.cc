#include "aos/events/logging/log_writer.h"

#include <dirent.h>

#include <functional>
#include <map>
#include <vector>

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/team_number.h"
#include "aos/network/timestamp_channel.h"

namespace aos {
namespace logger {
namespace {
using message_bridge::RemoteMessage;
namespace chrono = std::chrono;
}  // namespace

Logger::Logger(EventLoop *event_loop, const Configuration *configuration,
               std::function<bool(const Channel *)> should_log)
    : event_loop_(event_loop),
      configuration_(configuration),
      node_(configuration::GetNode(configuration_, event_loop->node())),
      node_index_(configuration::GetNodeIndex(configuration_, node_)),
      name_(network::GetHostname()),
      timer_handler_(event_loop_->AddTimer(
          [this]() { DoLogData(event_loop_->monotonic_now(), true); })),
      server_statistics_fetcher_(
          configuration::MultiNode(event_loop_->configuration())
              ? event_loop_->MakeFetcher<message_bridge::ServerStatistics>(
                    "/aos")
              : aos::Fetcher<message_bridge::ServerStatistics>()) {
  timer_handler_->set_name("channel_poll");
  VLOG(1) << "Creating logger for " << FlatbufferToJson(node_);

  // When we are logging remote timestamps, we need to be able to translate
  // from the channel index that the event loop uses to the channel index in
  // the config in the log file.
  event_loop_to_logged_channel_index_.resize(
      event_loop->configuration()->channels()->size(), -1);
  for (size_t event_loop_channel_index = 0;
       event_loop_channel_index <
       event_loop->configuration()->channels()->size();
       ++event_loop_channel_index) {
    const Channel *event_loop_channel =
        event_loop->configuration()->channels()->Get(event_loop_channel_index);

    const Channel *logged_channel = aos::configuration::GetChannel(
        configuration_, event_loop_channel->name()->string_view(),
        event_loop_channel->type()->string_view(), "", node_);

    if (logged_channel != nullptr) {
      event_loop_to_logged_channel_index_[event_loop_channel_index] =
          configuration::ChannelIndex(configuration_, logged_channel);
    }
  }

  // Map to match source channels with the timestamp logger, if the contents
  // should be reliable, and a list of all channels logged on it to be treated
  // as reliable.
  std::map<const Channel *, std::tuple<const Node *, bool, std::vector<bool>>>
      timestamp_logger_channels;

  message_bridge::ChannelTimestampFinder finder(event_loop_);
  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    if (!configuration::ChannelIsSendableOnNode(channel, event_loop_->node())) {
      continue;
    }
    if (!channel->has_destination_nodes()) {
      continue;
    }
    const size_t channel_index =
        configuration::ChannelIndex(event_loop_->configuration(), channel);

    for (const Connection *connection : *channel->destination_nodes()) {
      if (configuration::ConnectionDeliveryTimeIsLoggedOnNode(
              connection, event_loop_->node())) {
        const Node *other_node = configuration::GetNode(
            configuration_, connection->name()->string_view());

        VLOG(1) << "Timestamps are logged from "
                << FlatbufferToJson(other_node);
        // True if each channel's remote timestamps are split into a separate
        // RemoteMessage channel.
        const bool is_split =
            finder.SplitChannelForChannel(channel, connection) != nullptr;

        const Channel *const timestamp_logger_channel =
            finder.ForChannel(channel, connection);

        auto it = timestamp_logger_channels.find(timestamp_logger_channel);
        if (it != timestamp_logger_channels.end()) {
          CHECK(!is_split);
          CHECK_LT(channel_index, std::get<2>(it->second).size());
          std::get<2>(it->second)[channel_index] =
              (connection->time_to_live() == 0);
        } else {
          if (is_split) {
            timestamp_logger_channels.insert(std::make_pair(
                timestamp_logger_channel,
                std::make_tuple(other_node, (connection->time_to_live() == 0),
                                std::vector<bool>())));
          } else {
            std::vector<bool> channel_reliable_contents(
                event_loop->configuration()->channels()->size(), false);
            channel_reliable_contents[channel_index] =
                (connection->time_to_live() == 0);

            timestamp_logger_channels.insert(std::make_pair(
                timestamp_logger_channel,
                std::make_tuple(other_node, false,
                                std::move(channel_reliable_contents))));
          }
        }
      }
    }
  }

  for (size_t channel_index = 0;
       channel_index < configuration_->channels()->size(); ++channel_index) {
    const Channel *const config_channel =
        configuration_->channels()->Get(channel_index);
    // The MakeRawFetcher method needs a channel which is in the event loop
    // configuration() object, not the configuration_ object.  Go look that up
    // from the config.
    const Channel *channel = aos::configuration::GetChannel(
        event_loop_->configuration(), config_channel->name()->string_view(),
        config_channel->type()->string_view(), "", event_loop_->node());
    CHECK(channel != nullptr)
        << ": Failed to look up channel "
        << aos::configuration::CleanedChannelToString(config_channel);
    if (!should_log(config_channel)) {
      continue;
    }

    FetcherStruct fs;
    fs.channel_index = channel_index;
    fs.channel = channel;

    const bool is_local =
        configuration::ChannelIsSendableOnNode(config_channel, node_);

    const bool is_readable =
        configuration::ChannelIsReadableOnNode(config_channel, node_);
    const bool is_logged =
        configuration::ChannelMessageIsLoggedOnNode(config_channel, node_);
    const bool log_message = is_logged && is_readable;

    bool log_delivery_times = false;
    if (configuration::MultiNode(configuration_)) {
      const aos::Connection *connection =
          configuration::ConnectionToNode(config_channel, node_);

      log_delivery_times = configuration::ConnectionDeliveryTimeIsLoggedOnNode(
          connection, event_loop_->node());

      CHECK_EQ(log_delivery_times,
               configuration::ConnectionDeliveryTimeIsLoggedOnNode(
                   config_channel, node_, node_));

      if (connection) {
        fs.reliable_forwarding = (connection->time_to_live() == 0);
      }
    }

    // Now, detect a RemoteMessage timestamp logger where we should just log
    // the contents to a file directly.
    const bool log_contents = timestamp_logger_channels.find(channel) !=
                              timestamp_logger_channels.end();

    if (log_message || log_delivery_times || log_contents) {
      fs.fetcher = event_loop->MakeRawFetcher(channel);
      VLOG(1) << "Logging channel "
              << configuration::CleanedChannelToString(channel);

      if (log_delivery_times) {
        VLOG(1) << "  Delivery times";
        fs.wants_timestamp_writer = true;
        fs.timestamp_node_index = static_cast<int>(node_index_);
      }
      // Both the timestamp and data writers want data_node_index so it knows
      // what the source node is.
      if (log_message || log_delivery_times) {
        if (!is_local) {
          const Node *source_node = configuration::GetNode(
              configuration_, channel->source_node()->string_view());
          fs.data_node_index =
              configuration::GetNodeIndex(configuration_, source_node);
        }
      }
      if (log_message) {
        VLOG(1) << "  Data";
        fs.wants_writer = true;
        if (!is_local) {
          fs.log_type = LogType::kLogRemoteMessage;
        } else {
          fs.data_node_index = static_cast<int>(node_index_);
        }
      }
      if (log_contents) {
        VLOG(1) << "Timestamp logger channel "
                << configuration::CleanedChannelToString(channel);
        auto timestamp_logger_channel_info =
            timestamp_logger_channels.find(channel);
        CHECK(timestamp_logger_channel_info != timestamp_logger_channels.end());
        fs.timestamp_node = std::get<0>(timestamp_logger_channel_info->second);
        fs.reliable_contents =
            std::get<1>(timestamp_logger_channel_info->second);
        fs.channel_reliable_contents =
            std::get<2>(timestamp_logger_channel_info->second);
        fs.wants_contents_writer = true;
        fs.contents_node_index =
            configuration::GetNodeIndex(configuration_, fs.timestamp_node);
      }
      fetchers_.emplace_back(std::move(fs));
    }
  }
}

Logger::~Logger() {
  if (log_namer_) {
    // If we are replaying a log file, or in simulation, we want to force the
    // last bit of data to be logged.  The easiest way to deal with this is to
    // poll everything as we go to destroy the class, ie, shut down the
    // logger, and write it to disk.
    StopLogging(event_loop_->monotonic_now());
  }
}

bool Logger::RenameLogBase(std::string new_base_name) {
  // TODO(Naman): Got a crash in RenameLogBase. Putting in a CHECK_NOTNULL to
  // catch the bug if it happens again
  if (new_base_name == CHECK_NOTNULL(log_namer_)->base_name()) {
    return true;
  }
  log_namer_->set_base_name(new_base_name);
  Rotate();
  return true;
}

std::string Logger::WriteConfiguration(LogNamer *log_namer) {
  std::string config_sha256;

  if (separate_config_) {
    flatbuffers::FlatBufferBuilder fbb;
    flatbuffers::Offset<aos::Configuration> configuration_offset =
        CopyFlatBuffer(configuration_, &fbb);
    LogFileHeader::Builder log_file_header_builder(fbb);
    log_file_header_builder.add_configuration(configuration_offset);
    fbb.FinishSizePrefixed(log_file_header_builder.Finish());
    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> config_header(
        fbb.Release());
    config_sha256 = Sha256(config_header.span());
    LOG(INFO) << "Config sha256 of " << config_sha256;
    log_namer->WriteConfiguration(&config_header, config_sha256);
  }

  return config_sha256;
}

void Logger::StartLogging(std::unique_ptr<LogNamer> log_namer,
                          std::optional<UUID> log_start_uuid) {
  CHECK(!log_namer_) << ": Already logging";

  VLOG(1) << "Starting logger for " << FlatbufferToJson(node_);

  auto config_sha256 = WriteConfiguration(log_namer.get());

  log_namer_ = std::move(log_namer);

  log_event_uuid_ = UUID::Random();
  log_start_uuid_ = log_start_uuid;

  // We want to do as much work as possible before the initial Fetch. Time
  // between that and actually starting to log opens up the possibility of
  // falling off the end of the queue during that time.

  for (FetcherStruct &f : fetchers_) {
    if (f.wants_writer) {
      f.writer = log_namer_->MakeWriter(f.channel);
    }
    if (f.wants_timestamp_writer) {
      f.timestamp_writer = log_namer_->MakeTimestampWriter(f.channel);
    }
    if (f.wants_contents_writer) {
      f.contents_writer = log_namer_->MakeForwardedTimestampWriter(
          f.channel, CHECK_NOTNULL(f.timestamp_node));
    }
  }

  log_namer_->SetHeaderTemplate(MakeHeader(config_sha256));

  const aos::monotonic_clock::time_point beginning_time =
      event_loop_->monotonic_now();

  // Grab data from each channel right before we declare the log file started
  // so we can capture the latest message on each channel.  This lets us have
  // non periodic messages with configuration that now get logged.
  for (FetcherStruct &f : fetchers_) {
    const auto start = event_loop_->monotonic_now();
    const bool got_new = f.fetcher->Fetch();
    const auto end = event_loop_->monotonic_now();
    RecordFetchResult(start, end, got_new, &f);

    // If there is a message, we want to write it.
    f.written = f.fetcher->context().data == nullptr;
  }

  // Clear out any old timestamps in case we are re-starting logging.
  for (size_t i = 0; i < configuration::NodesCount(configuration_); ++i) {
    log_namer_->ClearStartTimes();
  }

  const aos::monotonic_clock::time_point fetch_time =
      event_loop_->monotonic_now();
  WriteHeader();
  const aos::monotonic_clock::time_point header_time =
      event_loop_->monotonic_now();

  VLOG(1) << "Logging node as " << FlatbufferToJson(node_) << " start_time "
          << last_synchronized_time_ << ", took "
          << chrono::duration<double>(fetch_time - beginning_time).count()
          << " to fetch, "
          << chrono::duration<double>(header_time - fetch_time).count()
          << " to write headers, boot uuid " << event_loop_->boot_uuid();

  // Force logging up until the start of the log file now, so the messages at
  // the start are always ordered before the rest of the messages.
  // Note: this ship may have already sailed, but we don't have to make it
  // worse.
  // TODO(austin): Test...
  //
  // This is safe to call here since we have set last_synchronized_time_ as
  // the same time as in the header, and all the data before it should be
  // logged without ordering concerns.
  LogUntil(last_synchronized_time_);

  timer_handler_->Setup(event_loop_->monotonic_now() + polling_period_,
                        polling_period_);
}

std::unique_ptr<LogNamer> Logger::RestartLogging(
    std::unique_ptr<LogNamer> log_namer, std::optional<UUID> log_start_uuid) {
  CHECK(log_namer_) << ": Unexpected restart while not logging";

  VLOG(1) << "Restarting logger for " << FlatbufferToJson(node_);

  // Force out every currently pending message, pointing all fetchers at the
  // last (currently available) records.  Note that LogUntil() updates
  // last_synchronized_time_ to the time value that it receives.
  while (LogUntil(last_synchronized_time_ + polling_period_))
    ;

  std::unique_ptr<LogNamer> old_log_namer = std::move(log_namer_);
  log_namer_ = std::move(log_namer);

  // Now grab a representative time on both the RT and monotonic clock.
  // Average a monotonic clock before and after to reduce the error.
  const aos::monotonic_clock::time_point beginning_time =
      event_loop_->monotonic_now();
  const aos::realtime_clock::time_point beginning_time_rt =
      event_loop_->realtime_now();
  const aos::monotonic_clock::time_point beginning_time2 =
      event_loop_->monotonic_now();

  if (beginning_time > last_synchronized_time_) {
    LOG(WARNING) << "Took over " << polling_period_.count()
                 << "ns to swap log_namer";
  }

  // Since we are going to log all in 1 big go, we need our log start time to
  // be after the previous LogUntil call finished, but before 1 period after
  // it. The best way to guarentee that is to pick a start time that is the
  // earliest of the two.  That covers the case where the OS puts us to sleep
  // between when we finish LogUntil and capture beginning_time.
  const aos::monotonic_clock::time_point monotonic_start_time =
      std::min(last_synchronized_time_, beginning_time);
  const aos::realtime_clock::time_point realtime_start_time =
      (beginning_time_rt + (monotonic_start_time.time_since_epoch() -
                            ((beginning_time.time_since_epoch() +
                              beginning_time2.time_since_epoch()) /
                             2)));

  auto config_sha256 = WriteConfiguration(log_namer_.get());

  log_event_uuid_ = UUID::Random();
  log_start_uuid_ = log_start_uuid;

  log_namer_->SetHeaderTemplate(MakeHeader(config_sha256));

  // Note that WriteHeader updates last_synchronized_time_ to be the
  // current time when it is called, which is then the "start time"
  // of the new (restarted) log. This timestamp will be after
  // the timestamp of the last message fetched on each channel, but is
  // carefully picked per the comment above to not violate
  // max_out_of_order_duration.
  WriteHeader(monotonic_start_time, realtime_start_time);

  const aos::monotonic_clock::time_point header_time =
      event_loop_->monotonic_now();

  // Write the transition record(s) for each channel ...
  for (FetcherStruct &f : fetchers_) {
    // Create writers from the new namer
    NewDataWriter *next_writer = nullptr;
    NewDataWriter *next_timestamp_writer = nullptr;
    NewDataWriter *next_contents_writer = nullptr;

    if (f.wants_writer) {
      next_writer = log_namer_->MakeWriter(f.channel);
    }
    if (f.wants_timestamp_writer) {
      next_timestamp_writer = log_namer_->MakeTimestampWriter(f.channel);
    }
    if (f.wants_contents_writer) {
      next_contents_writer = log_namer_->MakeForwardedTimestampWriter(
          f.channel, CHECK_NOTNULL(f.timestamp_node));
    }

    if (f.fetcher->context().data != nullptr) {
      // Write the last message fetched as the first of the new log of this
      // type. The timestamps on these will all be before the new start time.
      WriteData(next_writer, f);
      WriteTimestamps(next_timestamp_writer, f);
      WriteContent(next_contents_writer, f);

      // It is possible that a few more snuck in. Write them all out also,
      // including any that should also be in the old log.
      while (true) {
        // Get the next message ...
        const auto start = event_loop_->monotonic_now();
        const bool got_new = f.fetcher->FetchNext();
        const auto end = event_loop_->monotonic_now();
        RecordFetchResult(start, end, got_new, &f);

        if (got_new) {
          if (f.fetcher->context().monotonic_event_time <=
              last_synchronized_time_) {
            WriteFetchedRecord(f);
            WriteData(next_writer, f);
            WriteTimestamps(next_timestamp_writer, f);
            WriteContent(next_contents_writer, f);

          } else {
            f.written = false;
            break;
          }

        } else {
          f.written = true;
          break;
        }
      }
    }

    // Switch fully over to the new writers.
    f.writer = next_writer;
    f.timestamp_writer = next_timestamp_writer;
    f.contents_writer = next_contents_writer;
  }

  const aos::monotonic_clock::time_point channel_time =
      event_loop_->monotonic_now();

  VLOG(1) << "Logging node as " << FlatbufferToJson(node_) << " restart_time "
          << last_synchronized_time_ << ", took "
          << chrono::duration<double>(header_time - beginning_time).count()
          << " to prepare and write header, "
          << chrono::duration<double>(channel_time - header_time).count()
          << " to write initial channel messages, boot uuid "
          << event_loop_->boot_uuid();

  return old_log_namer;
}

std::unique_ptr<LogNamer> Logger::StopLogging(
    aos::monotonic_clock::time_point end_time) {
  CHECK(log_namer_) << ": Not logging right now";

  if (end_time != aos::monotonic_clock::min_time) {
    // Folks like to use the on_logged_period_ callback to trigger stop and
    // start events.  We can't have those then recurse and try to stop again.
    // Rather than making everything reentrant, let's just instead block the
    // callback here.
    DoLogData(end_time, false);
  }
  timer_handler_->Disable();

  for (FetcherStruct &f : fetchers_) {
    f.writer = nullptr;
    f.timestamp_writer = nullptr;
    f.contents_writer = nullptr;
  }

  log_event_uuid_ = UUID::Zero();
  log_start_uuid_ = std::nullopt;

  return std::move(log_namer_);
}

void Logger::WriteHeader(aos::monotonic_clock::time_point monotonic_start_time,
                         aos::realtime_clock::time_point realtime_start_time) {
  if (configuration::MultiNode(configuration_)) {
    server_statistics_fetcher_.Fetch();
  }

  if (monotonic_start_time == aos::monotonic_clock::min_time) {
    monotonic_start_time = event_loop_->monotonic_now();
    realtime_start_time = event_loop_->realtime_now();
  }

  // We need to pick a point in time to declare the log file "started".  This
  // starts here.  It needs to be after everything is fetched so that the
  // fetchers are all pointed at the most recent message before the start
  // time.
  last_synchronized_time_ = monotonic_start_time;

  for (const Node *node : log_namer_->nodes()) {
    const int node_index = configuration::GetNodeIndex(configuration_, node);
    MaybeUpdateTimestamp(node, node_index, monotonic_start_time,
                         realtime_start_time);
  }
}

void Logger::WriteMissingTimestamps() {
  if (configuration::MultiNode(configuration_)) {
    server_statistics_fetcher_.Fetch();
  } else {
    return;
  }

  if (server_statistics_fetcher_.get() == nullptr) {
    return;
  }

  for (const Node *node : log_namer_->nodes()) {
    const int node_index = configuration::GetNodeIndex(configuration_, node);
    if (MaybeUpdateTimestamp(
            node, node_index,
            server_statistics_fetcher_.context().monotonic_event_time,
            server_statistics_fetcher_.context().realtime_event_time)) {
      VLOG(1) << "Timestamps changed on " << aos::FlatbufferToJson(node);
    }
  }
}

bool Logger::MaybeUpdateTimestamp(
    const Node *node, int node_index,
    aos::monotonic_clock::time_point monotonic_start_time,
    aos::realtime_clock::time_point realtime_start_time) {
  // Bail early if the start times are already set.
  if (node_ == node || !configuration::MultiNode(configuration_)) {
    if (log_namer_->monotonic_start_time(node_index,
                                         event_loop_->boot_uuid()) !=
        monotonic_clock::min_time) {
      return false;
    }

    // There are no offsets to compute for ourself, so always succeed.
    log_namer_->SetStartTimes(node_index, event_loop_->boot_uuid(),
                              monotonic_start_time, realtime_start_time,
                              monotonic_start_time, realtime_start_time);
    return true;
  } else if (server_statistics_fetcher_.get() != nullptr) {
    // We must be a remote node now.  Look for the connection and see if it is
    // connected.
    CHECK(server_statistics_fetcher_->has_connections());

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

      CHECK(connection->has_boot_uuid());
      const UUID boot_uuid =
          UUID::FromString(connection->boot_uuid()->string_view());

      if (log_namer_->monotonic_start_time(node_index, boot_uuid) !=
          monotonic_clock::min_time) {
        break;
      }

      VLOG(1) << "Updating start time for "
              << aos::FlatbufferToJson(connection);

      // Found it and it is connected.  Compensate and go.
      log_namer_->SetStartTimes(
          node_index, boot_uuid,
          monotonic_start_time +
              std::chrono::nanoseconds(connection->monotonic_offset()),
          realtime_start_time, monotonic_start_time, realtime_start_time);
      return true;
    }
  }
  return false;
}

aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> Logger::MakeHeader(
    std::string_view config_sha256) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  flatbuffers::Offset<aos::Configuration> configuration_offset;
  if (!separate_config_) {
    configuration_offset = CopyFlatBuffer(configuration_, &fbb);
  } else {
    CHECK(!config_sha256.empty());
  }

  const flatbuffers::Offset<flatbuffers::String> name_offset =
      fbb.CreateString(name_);

  const flatbuffers::Offset<flatbuffers::String> logger_sha1_offset =
      logger_sha1_.empty() ? 0 : fbb.CreateString(logger_sha1_);
  const flatbuffers::Offset<flatbuffers::String> logger_version_offset =
      logger_version_.empty() ? 0 : fbb.CreateString(logger_version_);

  CHECK(log_event_uuid_ != UUID::Zero());
  const flatbuffers::Offset<flatbuffers::String> log_event_uuid_offset =
      log_event_uuid_.PackString(&fbb);

  const flatbuffers::Offset<flatbuffers::String> logger_instance_uuid_offset =
      logger_instance_uuid_.PackString(&fbb);

  flatbuffers::Offset<flatbuffers::String> log_start_uuid_offset;
  if (log_start_uuid_) {
    log_start_uuid_offset = fbb.CreateString(log_start_uuid_->ToString());
  }

  flatbuffers::Offset<flatbuffers::String> config_sha256_offset;
  if (!config_sha256.empty()) {
    config_sha256_offset = fbb.CreateString(config_sha256);
  }

  const flatbuffers::Offset<flatbuffers::String> logger_node_boot_uuid_offset =
      event_loop_->boot_uuid().PackString(&fbb);

  flatbuffers::Offset<Node> logger_node_offset;

  if (configuration::MultiNode(configuration_)) {
    logger_node_offset = RecursiveCopyFlatBuffer(node_, &fbb);
  }

  aos::logger::LogFileHeader::Builder log_file_header_builder(fbb);

  log_file_header_builder.add_name(name_offset);
  if (!logger_sha1_offset.IsNull()) {
    log_file_header_builder.add_logger_sha1(logger_sha1_offset);
  }
  if (!logger_version_offset.IsNull()) {
    log_file_header_builder.add_logger_version(logger_version_offset);
  }

  // Only add the node if we are running in a multinode configuration.
  if (configuration::MultiNode(configuration_)) {
    log_file_header_builder.add_logger_node(logger_node_offset);
  }

  if (!configuration_offset.IsNull()) {
    log_file_header_builder.add_configuration(configuration_offset);
  }
  // The worst case theoretical out of order is the polling period times 2.
  // One message could get logged right after the boundary, but be for right
  // before the next boundary.  And the reverse could happen for another
  // message.  Report back 3x to be extra safe, and because the cost isn't
  // huge on the read side.
  log_file_header_builder.add_max_out_of_order_duration(
      std::chrono::nanoseconds(3 * polling_period_).count());

  log_file_header_builder.add_log_event_uuid(log_event_uuid_offset);
  log_file_header_builder.add_logger_instance_uuid(logger_instance_uuid_offset);
  if (!log_start_uuid_offset.IsNull()) {
    log_file_header_builder.add_log_start_uuid(log_start_uuid_offset);
  }
  log_file_header_builder.add_logger_node_boot_uuid(
      logger_node_boot_uuid_offset);

  if (!config_sha256_offset.IsNull()) {
    log_file_header_builder.add_configuration_sha256(config_sha256_offset);
  }

  fbb.FinishSizePrefixed(log_file_header_builder.Finish());
  aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> result(
      fbb.Release());

  CHECK(result.Verify()) << ": Built a corrupted header.";

  return result;
}

void Logger::ResetStatisics() {
  max_message_fetch_time_ = std::chrono::nanoseconds::zero();
  max_message_fetch_time_channel_ = -1;
  max_message_fetch_time_size_ = -1;
  total_message_fetch_time_ = std::chrono::nanoseconds::zero();
  total_message_fetch_count_ = 0;
  total_message_fetch_bytes_ = 0;
  total_nop_fetch_time_ = std::chrono::nanoseconds::zero();
  total_nop_fetch_count_ = 0;
  max_copy_time_ = std::chrono::nanoseconds::zero();
  max_copy_time_channel_ = -1;
  max_copy_time_size_ = -1;
  total_copy_time_ = std::chrono::nanoseconds::zero();
  total_copy_count_ = 0;
  total_copy_bytes_ = 0;
}

void Logger::Rotate() {
  for (const Node *node : log_namer_->nodes()) {
    log_namer_->Rotate(node);
  }
}

void Logger::WriteData(NewDataWriter *writer, const FetcherStruct &f) {
  if (writer != nullptr) {
    const UUID source_node_boot_uuid =
        static_cast<int>(node_index_) != f.data_node_index
            ? f.fetcher->context().source_boot_uuid
            : event_loop_->boot_uuid();
    // Write!
    const auto start = event_loop_->monotonic_now();

    ContextDataCopier coppier(f.fetcher->context(), f.channel_index, f.log_type,
                              event_loop_);

    writer->CopyMessage(&coppier, source_node_boot_uuid, start);
    RecordCreateMessageTime(start, coppier.end_time(), f);

    VLOG(2) << "Wrote data as node " << FlatbufferToJson(node_)
            << " for channel "
            << configuration::CleanedChannelToString(f.fetcher->channel())
            << " to " << writer->filename();
  }
}

void Logger::WriteTimestamps(NewDataWriter *timestamp_writer,
                             const FetcherStruct &f) {
  if (timestamp_writer != nullptr) {
    // And now handle timestamps.

    // Tell our writer that we know something about the remote boot.
    timestamp_writer->UpdateRemote(
        f.data_node_index, f.fetcher->context().source_boot_uuid,
        f.fetcher->context().monotonic_remote_time,
        f.fetcher->context().monotonic_event_time, f.reliable_forwarding);

    const auto start = event_loop_->monotonic_now();
    ContextDataCopier coppier(f.fetcher->context(), f.channel_index,
                              LogType::kLogDeliveryTimeOnly, event_loop_);

    timestamp_writer->CopyMessage(&coppier, event_loop_->boot_uuid(), start);
    RecordCreateMessageTime(start, coppier.end_time(), f);

    VLOG(2) << "Wrote timestamps as node " << FlatbufferToJson(node_)
            << " for channel "
            << configuration::CleanedChannelToString(f.fetcher->channel())
            << " to " << timestamp_writer->filename() << " timestamp";
  }
}

void Logger::WriteContent(NewDataWriter *contents_writer,
                          const FetcherStruct &f) {
  if (contents_writer != nullptr) {
    const auto start = event_loop_->monotonic_now();
    // And now handle the special message contents channel.  Copy the
    // message into a FlatBufferBuilder and save it to disk.
    const RemoteMessage *msg =
        flatbuffers::GetRoot<RemoteMessage>(f.fetcher->context().data);

    CHECK(msg->has_boot_uuid()) << ": " << aos::FlatbufferToJson(msg);
    // Translate from the channel index that the event loop uses to the
    // channel index in the log file.
    const int channel_index =
        event_loop_to_logged_channel_index_[msg->channel_index()];

    const aos::monotonic_clock::time_point monotonic_timestamp_time =
        f.fetcher->context().monotonic_event_time;

    // Timestamps tell us information about what happened too!
    // Capture any reboots so UpdateRemote is properly recorded.
    contents_writer->UpdateBoot(UUID::FromVector(msg->boot_uuid()));

    // Start with recording info about the data flowing from our node to the
    // remote.
    const bool reliable =
        f.channel_reliable_contents.size() != 0u
            ? f.channel_reliable_contents[msg->channel_index()]
            : f.reliable_contents;

    contents_writer->UpdateRemote(
        node_index_, event_loop_->boot_uuid(),
        monotonic_clock::time_point(
            chrono::nanoseconds(msg->monotonic_remote_time())),
        monotonic_clock::time_point(
            chrono::nanoseconds(msg->monotonic_sent_time())),
        reliable, monotonic_timestamp_time);

    RemoteMessageCopier coppier(msg, channel_index, monotonic_timestamp_time,
                                event_loop_);

    contents_writer->CopyMessage(&coppier, UUID::FromVector(msg->boot_uuid()),
                                 start);

    RecordCreateMessageTime(start, coppier.end_time(), f);
  }
}

void Logger::WriteFetchedRecord(FetcherStruct &f) {
  WriteData(f.writer, f);
  WriteTimestamps(f.timestamp_writer, f);
  WriteContent(f.contents_writer, f);
}

bool Logger::LogUntil(monotonic_clock::time_point t) {
  bool has_pending_messages = false;

  // Grab the latest ServerStatistics message.  This will always have the
  // oppertunity to be >= to the current time, so it will always represent any
  // reboots which may have happened.
  WriteMissingTimestamps();

  // Write each channel to disk, one at a time.
  for (FetcherStruct &f : fetchers_) {
    while (true) {
      if (f.written) {
        const auto start = event_loop_->monotonic_now();
        const bool got_new = f.fetcher->FetchNext();
        const auto end = event_loop_->monotonic_now();
        RecordFetchResult(start, end, got_new, &f);
        if (!got_new) {
          VLOG(2) << "No new data on "
                  << configuration::CleanedChannelToString(
                         f.fetcher->channel());
          break;
        }
        f.written = false;
      }

      // TODO(james): Write tests to exercise this logic.
      if (f.fetcher->context().monotonic_event_time >= t) {
        has_pending_messages = true;
        break;
      }

      WriteFetchedRecord(f);

      f.written = true;
    }
  }
  last_synchronized_time_ = t;

  return has_pending_messages;
}

void Logger::DoLogData(const monotonic_clock::time_point end_time,
                       bool run_on_logged) {
  // We want to guarantee that messages aren't out of order by more than
  // max_out_of_order_duration.  To do this, we need sync points.  Every write
  // cycle should be a sync point.

  do {
    // Move the sync point up by at most polling_period.  This forces one sync
    // per iteration, even if it is small.
    LogUntil(std::min(last_synchronized_time_ + polling_period_, end_time));

    if (run_on_logged) {
      on_logged_period_();
    }

    // If we missed cycles, we could be pretty far behind.  Spin until we are
    // caught up.
  } while (last_synchronized_time_ + polling_period_ < end_time);
}

void Logger::RecordFetchResult(aos::monotonic_clock::time_point start,
                               aos::monotonic_clock::time_point end,
                               bool got_new, FetcherStruct *fetcher) {
  const auto duration = end - start;
  if (!got_new) {
    ++total_nop_fetch_count_;
    total_nop_fetch_time_ += duration;
    return;
  }
  ++total_message_fetch_count_;
  total_message_fetch_bytes_ += fetcher->fetcher->context().size;
  total_message_fetch_time_ += duration;
  if (duration > max_message_fetch_time_) {
    max_message_fetch_time_ = duration;
    max_message_fetch_time_channel_ = fetcher->channel_index;
    max_message_fetch_time_size_ = fetcher->fetcher->context().size;
  }
}

void Logger::RecordCreateMessageTime(aos::monotonic_clock::time_point start,
                                     aos::monotonic_clock::time_point end,
                                     const FetcherStruct &fetcher) {
  const auto duration = end - start;
  total_copy_time_ += duration;
  ++total_copy_count_;
  total_copy_bytes_ += fetcher.fetcher->context().size;
  if (duration > max_copy_time_) {
    max_copy_time_ = duration;
    max_copy_time_channel_ = fetcher.channel_index;
    max_copy_time_size_ = fetcher.fetcher->context().size;
  }
}

}  // namespace logger
}  // namespace aos
