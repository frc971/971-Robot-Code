#include "aos/events/logging/log_writer.h"

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
      name_(network::GetHostname()),
      timer_handler_(event_loop_->AddTimer(
          [this]() { DoLogData(event_loop_->monotonic_now()); })),
      server_statistics_fetcher_(
          configuration::MultiNode(event_loop_->configuration())
              ? event_loop_->MakeFetcher<message_bridge::ServerStatistics>(
                    "/aos")
              : aos::Fetcher<message_bridge::ServerStatistics>()) {
  VLOG(1) << "Creating logger for " << FlatbufferToJson(event_loop_->node());

  std::map<const Channel *, const Node *> timestamp_logger_channels;

  message_bridge::ChannelTimestampFinder finder(event_loop_);
  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    if (!configuration::ChannelIsSendableOnNode(channel, event_loop_->node())) {
      continue;
    }
    if (!channel->has_destination_nodes()) {
      continue;
    }
    for (const Connection *connection : *channel->destination_nodes()) {
      if (configuration::ConnectionDeliveryTimeIsLoggedOnNode(
              connection, event_loop_->node())) {
        const Node *other_node = configuration::GetNode(
            event_loop_->configuration(), connection->name()->string_view());

        VLOG(1) << "Timestamps are logged from "
                << FlatbufferToJson(other_node);
        timestamp_logger_channels.insert(
            std::make_pair(finder.ForChannel(channel, connection), other_node));
      }
    }
  }

  const size_t our_node_index =
      configuration::GetNodeIndex(configuration_, event_loop_->node());

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
    if (!should_log(channel)) {
      continue;
    }

    FetcherStruct fs;
    fs.channel_index = channel_index;
    fs.channel = channel;

    const bool is_local =
        configuration::ChannelIsSendableOnNode(channel, event_loop_->node());

    const bool is_readable =
        configuration::ChannelIsReadableOnNode(channel, event_loop_->node());
    const bool is_logged = configuration::ChannelMessageIsLoggedOnNode(
        channel, event_loop_->node());
    const bool log_message = is_logged && is_readable;

    bool log_delivery_times = false;
    if (event_loop_->node() != nullptr) {
      log_delivery_times = configuration::ConnectionDeliveryTimeIsLoggedOnNode(
          channel, event_loop_->node(), event_loop_->node());
    }

    // Now, detect a RemoteMessage timestamp logger where we should just log the
    // contents to a file directly.
    const bool log_contents = timestamp_logger_channels.find(channel) !=
                              timestamp_logger_channels.end();

    if (log_message || log_delivery_times || log_contents) {
      fs.fetcher = event_loop->MakeRawFetcher(channel);
      VLOG(1) << "Logging channel "
              << configuration::CleanedChannelToString(channel);

      if (log_delivery_times) {
        VLOG(1) << "  Delivery times";
        fs.wants_timestamp_writer = true;
        fs.timestamp_node_index = our_node_index;
      }
      if (log_message) {
        VLOG(1) << "  Data";
        fs.wants_writer = true;
        if (!is_local) {
          const Node *source_node = configuration::GetNode(
              configuration_, channel->source_node()->string_view());
          fs.data_node_index =
              configuration::GetNodeIndex(configuration_, source_node);
          fs.log_type = LogType::kLogRemoteMessage;
        } else {
          fs.data_node_index = our_node_index;
        }
      }
      if (log_contents) {
        VLOG(1) << "Timestamp logger channel "
                << configuration::CleanedChannelToString(channel);
        fs.timestamp_node = timestamp_logger_channels.find(channel)->second;
        fs.wants_contents_writer = true;
        fs.contents_node_index =
            configuration::GetNodeIndex(configuration_, fs.timestamp_node);
      }
      fetchers_.emplace_back(std::move(fs));
    }
  }

  // When we are logging remote timestamps, we need to be able to translate from
  // the channel index that the event loop uses to the channel index in the
  // config in the log file.
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
        event_loop_channel->type()->string_view(), "",
        configuration::GetNode(configuration_, event_loop_->node()));

    if (logged_channel != nullptr) {
      event_loop_to_logged_channel_index_[event_loop_channel_index] =
          configuration::ChannelIndex(configuration_, logged_channel);
    }
  }
}

Logger::~Logger() {
  if (log_namer_) {
    // If we are replaying a log file, or in simulation, we want to force the
    // last bit of data to be logged.  The easiest way to deal with this is to
    // poll everything as we go to destroy the class, ie, shut down the logger,
    // and write it to disk.
    StopLogging(event_loop_->monotonic_now());
  }
}

void Logger::StartLogging(std::unique_ptr<LogNamer> log_namer,
                          std::string_view log_start_uuid) {
  CHECK(!log_namer_) << ": Already logging";
  log_namer_ = std::move(log_namer);

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
    log_namer_->WriteConfiguration(&config_header, config_sha256);
  }

  log_event_uuid_ = UUID::Random();
  log_start_uuid_ = log_start_uuid;
  VLOG(1) << "Starting logger for " << FlatbufferToJson(event_loop_->node());

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

  CHECK(node_state_.empty());
  node_state_.resize(configuration::MultiNode(configuration_)
                         ? configuration_->nodes()->size()
                         : 1u);

  for (const Node *node : log_namer_->nodes()) {
    const int node_index = configuration::GetNodeIndex(configuration_, node);

    node_state_[node_index].log_file_header = MakeHeader(node, config_sha256);
  }

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
  for (size_t i = 0; i < node_state_.size(); ++i) {
    SetStartTime(i, monotonic_clock::min_time, realtime_clock::min_time,
                 monotonic_clock::min_time, realtime_clock::min_time);
  }

  WriteHeader();

  LOG(INFO) << "Logging node as " << FlatbufferToJson(event_loop_->node())
            << " start_time " << last_synchronized_time_ << " boot uuid "
            << event_loop_->boot_uuid();

  // Force logging up until the start of the log file now, so the messages at
  // the start are always ordered before the rest of the messages.
  // Note: this ship may have already sailed, but we don't have to make it
  // worse.
  // TODO(austin): Test...
  //
  // This is safe to call here since we have set last_synchronized_time_ as the
  // same time as in the header, and all the data before it should be logged
  // without ordering concerns.
  LogUntil(last_synchronized_time_);

  timer_handler_->Setup(event_loop_->monotonic_now() + polling_period_,
                        polling_period_);
}

std::unique_ptr<LogNamer> Logger::StopLogging(
    aos::monotonic_clock::time_point end_time) {
  CHECK(log_namer_) << ": Not logging right now";

  if (end_time != aos::monotonic_clock::min_time) {
    DoLogData(end_time);
  }
  timer_handler_->Disable();

  for (FetcherStruct &f : fetchers_) {
    f.writer = nullptr;
    f.timestamp_writer = nullptr;
    f.contents_writer = nullptr;
  }
  node_state_.clear();

  log_event_uuid_ = UUID::Zero();
  log_start_uuid_ = std::string();

  return std::move(log_namer_);
}

void Logger::WriteHeader() {
  if (configuration::MultiNode(configuration_)) {
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
    const int node_index = configuration::GetNodeIndex(configuration_, node);
    MaybeUpdateTimestamp(node, node_index, monotonic_start_time,
                         realtime_start_time);
    MaybeWriteHeader(node_index, node);
  }
}

void Logger::MaybeWriteHeader(int node_index) {
  if (configuration::MultiNode(configuration_)) {
    return MaybeWriteHeader(node_index,
                            configuration_->nodes()->Get(node_index));
  } else {
    return MaybeWriteHeader(node_index, nullptr);
  }
}

void Logger::MaybeWriteHeader(int node_index, const Node *node) {
  // This function is responsible for writing the header when the header both
  // has valid data, and when it needs to be written.
  if (node_state_[node_index].header_written &&
      node_state_[node_index].header_valid) {
    // The header has been written and is valid, nothing to do.
    return;
  }
  if (!node_state_[node_index].has_source_node_boot_uuid) {
    // Can't write a header if we don't have the boot UUID.
    return;
  }

  // WriteHeader writes the first header in a log file.  We want to do this only
  // once.
  //
  // Rotate rewrites the same header with a new part ID, but keeps the same part
  // UUID.  We don't want that when things reboot, because that implies that
  // parts go together across a reboot.
  //
  // Reboot resets the parts UUID.  So, once we've written a header the first
  // time, we want to use Reboot to rotate the log and reset the parts UUID.
  //
  // header_valid is cleared whenever the remote reboots.
  if (node_state_[node_index].header_written) {
    log_namer_->Reboot(node, &node_state_[node_index].log_file_header);
  } else {
    log_namer_->WriteHeader(&node_state_[node_index].log_file_header, node);

    node_state_[node_index].header_written = true;
  }
  node_state_[node_index].header_valid = true;
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
      CHECK(node_state_[node_index].header_written);
      CHECK(node_state_[node_index].header_valid);
      log_namer_->Rotate(node, &node_state_[node_index].log_file_header);
    } else {
      MaybeWriteHeader(node_index, node);
    }
  }
}

void Logger::SetStartTime(
    size_t node_index, aos::monotonic_clock::time_point monotonic_start_time,
    aos::realtime_clock::time_point realtime_start_time,
    aos::monotonic_clock::time_point logger_monotonic_start_time,
    aos::realtime_clock::time_point logger_realtime_start_time) {
  node_state_[node_index].monotonic_start_time = monotonic_start_time;
  node_state_[node_index].realtime_start_time = realtime_start_time;
  node_state_[node_index]
      .log_file_header.mutable_message()
      ->mutate_monotonic_start_time(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              monotonic_start_time.time_since_epoch())
              .count());

  // Add logger start times if they are available in the log file header.
  if (node_state_[node_index]
          .log_file_header.mutable_message()
          ->has_logger_monotonic_start_time()) {
    node_state_[node_index]
        .log_file_header.mutable_message()
        ->mutate_logger_monotonic_start_time(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                logger_monotonic_start_time.time_since_epoch())
                .count());
  }

  if (node_state_[node_index]
          .log_file_header.mutable_message()
          ->has_logger_realtime_start_time()) {
    node_state_[node_index]
        .log_file_header.mutable_message()
        ->mutate_logger_realtime_start_time(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                logger_realtime_start_time.time_since_epoch())
                .count());
  }

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
  // Bail early if the start times are already set.
  if (node_state_[node_index].monotonic_start_time !=
      monotonic_clock::min_time) {
    return false;
  }
  if (event_loop_->node() == node ||
      !configuration::MultiNode(configuration_)) {
    // There are no offsets to compute for ourself, so always succeed.
    SetStartTime(node_index, monotonic_start_time, realtime_start_time,
                 monotonic_start_time, realtime_start_time);
    node_state_[node_index].SetBootUUID(event_loop_->boot_uuid());
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

      // Found it and it is connected.  Compensate and go.
      SetStartTime(node_index,
                   monotonic_start_time +
                       std::chrono::nanoseconds(connection->monotonic_offset()),
                   realtime_start_time, monotonic_start_time,
                   realtime_start_time);
      return true;
    }
  }
  return false;
}

aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> Logger::MakeHeader(
    const Node *node, std::string_view config_sha256) {
  // Now write the header with this timestamp in it.
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

  CHECK(log_event_uuid_ != UUID::Zero());
  const flatbuffers::Offset<flatbuffers::String> log_event_uuid_offset =
      log_event_uuid_.PackString(&fbb);

  const flatbuffers::Offset<flatbuffers::String> logger_instance_uuid_offset =
      logger_instance_uuid_.PackString(&fbb);

  flatbuffers::Offset<flatbuffers::String> log_start_uuid_offset;
  if (!log_start_uuid_.empty()) {
    log_start_uuid_offset = fbb.CreateString(log_start_uuid_);
  }

  flatbuffers::Offset<flatbuffers::String> config_sha256_offset;
  if (!config_sha256.empty()) {
    config_sha256_offset = fbb.CreateString(config_sha256);
  }

  const flatbuffers::Offset<flatbuffers::String> logger_node_boot_uuid_offset =
      event_loop_->boot_uuid().PackString(&fbb);

  const flatbuffers::Offset<flatbuffers::String> source_node_boot_uuid_offset =
      event_loop_->boot_uuid().PackString(&fbb);

  const flatbuffers::Offset<flatbuffers::String> parts_uuid_offset =
      fbb.CreateString("00000000-0000-4000-8000-000000000000");

  flatbuffers::Offset<Node> node_offset;
  flatbuffers::Offset<Node> logger_node_offset;

  if (configuration::MultiNode(configuration_)) {
    node_offset = RecursiveCopyFlatBuffer(node, &fbb);
    logger_node_offset = RecursiveCopyFlatBuffer(event_loop_->node(), &fbb);
  }

  aos::logger::LogFileHeader::Builder log_file_header_builder(fbb);

  log_file_header_builder.add_name(name_offset);

  // Only add the node if we are running in a multinode configuration.
  if (node != nullptr) {
    log_file_header_builder.add_node(node_offset);
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

  log_file_header_builder.add_monotonic_start_time(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          monotonic_clock::min_time.time_since_epoch())
          .count());
  if (node == event_loop_->node()) {
    log_file_header_builder.add_realtime_start_time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            realtime_clock::min_time.time_since_epoch())
            .count());
  } else {
    log_file_header_builder.add_logger_monotonic_start_time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            monotonic_clock::min_time.time_since_epoch())
            .count());
    log_file_header_builder.add_logger_realtime_start_time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            realtime_clock::min_time.time_since_epoch())
            .count());
  }

  log_file_header_builder.add_log_event_uuid(log_event_uuid_offset);
  log_file_header_builder.add_logger_instance_uuid(logger_instance_uuid_offset);
  if (!log_start_uuid_offset.IsNull()) {
    log_file_header_builder.add_log_start_uuid(log_start_uuid_offset);
  }
  log_file_header_builder.add_logger_node_boot_uuid(
      logger_node_boot_uuid_offset);
  log_file_header_builder.add_source_node_boot_uuid(
      source_node_boot_uuid_offset);

  log_file_header_builder.add_parts_uuid(parts_uuid_offset);
  log_file_header_builder.add_parts_index(0);

  log_file_header_builder.add_configuration_sha256(0);

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
    const int node_index = configuration::GetNodeIndex(configuration_, node);
    log_namer_->Rotate(node, &node_state_[node_index].log_file_header);
  }
}

void Logger::LogUntil(monotonic_clock::time_point t) {
  // Grab the latest ServerStatistics message.  This will always have the
  // oppertunity to be >= to the current time, so it will always represent any
  // reboots which may have happened.
  WriteMissingTimestamps();

  int our_node_index = aos::configuration::GetNodeIndex(
      event_loop_->configuration(), event_loop_->node());

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
        break;
      }
      if (f.writer != nullptr) {
        // Only check if the boot UUID has changed if this is data from another
        // node.  Our UUID can't change without restarting the application.
        if (our_node_index != f.data_node_index) {
          // And update our boot UUID if the UUID has changed.
          if (node_state_[f.data_node_index].SetBootUUID(
                  f.fetcher->context().remote_boot_uuid)) {
            MaybeWriteHeader(f.data_node_index);
          }
        }

        // Write!
        const auto start = event_loop_->monotonic_now();
        flatbuffers::FlatBufferBuilder fbb(f.fetcher->context().size +
                                           max_header_size_);
        fbb.ForceDefaults(true);

        fbb.FinishSizePrefixed(PackMessage(&fbb, f.fetcher->context(),
                                           f.channel_index, f.log_type));
        const auto end = event_loop_->monotonic_now();
        RecordCreateMessageTime(start, end, &f);

        VLOG(2) << "Writing data as node "
                << FlatbufferToJson(event_loop_->node()) << " for channel "
                << configuration::CleanedChannelToString(f.fetcher->channel())
                << " to " << f.writer->filename() << " data "
                << FlatbufferToJson(
                       flatbuffers::GetSizePrefixedRoot<MessageHeader>(
                           fbb.GetBufferPointer()));

        max_header_size_ = std::max(max_header_size_,
                                    fbb.GetSize() - f.fetcher->context().size);
        CHECK(node_state_[f.data_node_index].header_valid)
            << ": Can't write data before the header on channel "
            << configuration::CleanedChannelToString(f.fetcher->channel());
        f.writer->QueueSizedFlatbuffer(&fbb, end);
      }

      if (f.timestamp_writer != nullptr) {
        // And now handle timestamps.
        const auto start = event_loop_->monotonic_now();
        flatbuffers::FlatBufferBuilder fbb;
        fbb.ForceDefaults(true);

        fbb.FinishSizePrefixed(PackMessage(&fbb, f.fetcher->context(),
                                           f.channel_index,
                                           LogType::kLogDeliveryTimeOnly));
        const auto end = event_loop_->monotonic_now();
        RecordCreateMessageTime(start, end, &f);

        VLOG(2) << "Writing timestamps as node "
                << FlatbufferToJson(event_loop_->node()) << " for channel "
                << configuration::CleanedChannelToString(f.fetcher->channel())
                << " to " << f.timestamp_writer->filename() << " timestamp "
                << FlatbufferToJson(
                       flatbuffers::GetSizePrefixedRoot<MessageHeader>(
                           fbb.GetBufferPointer()));

        CHECK(node_state_[f.timestamp_node_index].header_valid)
            << ": Can't write data before the header on channel "
            << configuration::CleanedChannelToString(f.fetcher->channel());
        f.timestamp_writer->QueueSizedFlatbuffer(&fbb, end);
      }

      if (f.contents_writer != nullptr) {
        const auto start = event_loop_->monotonic_now();
        // And now handle the special message contents channel.  Copy the
        // message into a FlatBufferBuilder and save it to disk.
        // TODO(austin): We can be more efficient here when we start to
        // care...
        flatbuffers::FlatBufferBuilder fbb;
        fbb.ForceDefaults(true);

        const RemoteMessage *msg =
            flatbuffers::GetRoot<RemoteMessage>(f.fetcher->context().data);

        CHECK(msg->has_boot_uuid()) << ": " << aos::FlatbufferToJson(msg);
        if (node_state_[f.contents_node_index].SetBootUUID(
                UUID::FromVector(msg->boot_uuid()))) {
          MaybeWriteHeader(f.contents_node_index);
        }

        logger::MessageHeader::Builder message_header_builder(fbb);

        // TODO(austin): This needs to check the channel_index and confirm
        // that it should be logged before squirreling away the timestamp to
        // disk.  We don't want to log irrelevant timestamps.

        // Note: this must match the same order as MessageBridgeServer and
        // PackMessage.  We want identical headers to have identical
        // on-the-wire formats to make comparing them easier.

        // Translate from the channel index that the event loop uses to the
        // channel index in the log file.
        message_header_builder.add_channel_index(
            event_loop_to_logged_channel_index_[msg->channel_index()]);

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

        message_header_builder.add_monotonic_timestamp_time(
            f.fetcher->context()
                .monotonic_event_time.time_since_epoch()
                .count());

        fbb.FinishSizePrefixed(message_header_builder.Finish());
        const auto end = event_loop_->monotonic_now();
        RecordCreateMessageTime(start, end, &f);

        CHECK(node_state_[f.contents_node_index].header_valid)
            << ": Can't write data before the header on channel "
            << configuration::CleanedChannelToString(f.fetcher->channel());
        f.contents_writer->QueueSizedFlatbuffer(&fbb, end);
      }

      f.written = true;
    }
  }
  last_synchronized_time_ = t;
}

void Logger::DoLogData(const monotonic_clock::time_point end_time) {
  // We want to guarantee that messages aren't out of order by more than
  // max_out_of_order_duration.  To do this, we need sync points.  Every write
  // cycle should be a sync point.

  do {
    // Move the sync point up by at most polling_period.  This forces one sync
    // per iteration, even if it is small.
    LogUntil(std::min(last_synchronized_time_ + polling_period_, end_time));

    on_logged_period_();

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
                                     FetcherStruct *fetcher) {
  const auto duration = end - start;
  total_copy_time_ += duration;
  ++total_copy_count_;
  total_copy_bytes_ += fetcher->fetcher->context().size;
  if (duration > max_copy_time_) {
    max_copy_time_ = duration;
    max_copy_time_channel_ = fetcher->channel_index;
    max_copy_time_size_ = fetcher->fetcher->context().size;
  }
}

}  // namespace logger
}  // namespace aos
