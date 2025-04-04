#include "aos/events/logging/log_namer.h"

#include <functional>
#include <map>
#include <memory>
#include <string_view>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_cat.h"
#include "flatbuffers/flatbuffers.h"

#include "aos/containers/error_list.h"
#include "aos/containers/sized_array.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/flatbuffer_merge.h"
#include "aos/uuid.h"

ABSL_DECLARE_FLAG(int32_t, flush_size);

namespace aos::logger {

NewDataWriter::NewDataWriter(LogNamer *log_namer, const Node *node,
                             const Node *logger_node,
                             std::function<void(NewDataWriter *)> reopen,
                             std::function<void(NewDataWriter *)> close,
                             size_t max_message_size,
                             std::initializer_list<StoredDataType> types)
    : node_(node),
      node_index_(configuration::GetNodeIndex(log_namer->configuration_, node)),
      logger_node_index_(
          configuration::GetNodeIndex(log_namer->configuration_, logger_node)),
      log_namer_(log_namer),
      reopen_(std::move(reopen)),
      close_(std::move(close)),
      max_message_size_(max_message_size),
      max_out_of_order_duration_(log_namer_->base_max_out_of_order_duration()) {
  allowed_data_types_.fill(false);

  state_.resize(configuration::NodesCount(log_namer->configuration_));
  CHECK_LT(node_index_, state_.size());
  for (StoredDataType type : types) {
    CHECK_LT(static_cast<size_t>(type), allowed_data_types_.size());
    allowed_data_types_[static_cast<size_t>(type)] = true;
  }
}

NewDataWriter::~NewDataWriter() {
  if (writer) {
    Close();
  }
}

void NewDataWriter::Rotate() {
  // No need to rotate if nothing has been written.
  if (header_written_) {
    VLOG(1) << "Rotated " << name();
    ++parts_index_;

    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> header =
        MakeHeader();

    if (header.span().size() > max_message_size_) {
      max_message_size_ = header.span().size();
    }

    reopen_(this);
    header_written_ = false;
    QueueHeader(std::move(header));
  }
}

void NewDataWriter::Reboot(const UUID &source_node_boot_uuid) {
  parts_uuid_ = UUID::Random();
  ++parts_index_;
  reopen_(this);
  header_written_ = false;
  for (State &state : state_) {
    state.boot_uuid = UUID::Zero();
    state.oldest_remote_monotonic_timestamp = monotonic_clock::max_time;
    state.oldest_local_monotonic_timestamp = monotonic_clock::max_time;
    state.oldest_remote_unreliable_monotonic_timestamp =
        monotonic_clock::max_time;
    state.oldest_local_unreliable_monotonic_timestamp =
        monotonic_clock::max_time;
    state.oldest_remote_reliable_monotonic_timestamp =
        monotonic_clock::max_time;
    state.oldest_local_reliable_monotonic_timestamp = monotonic_clock::max_time;
    state.oldest_logger_remote_unreliable_monotonic_timestamp =
        monotonic_clock::max_time;
    state.oldest_logger_local_unreliable_monotonic_timestamp =
        monotonic_clock::max_time;
    state.oldest_remote_reliable_monotonic_transmit_timestamp =
        monotonic_clock::max_time;
    state.oldest_local_reliable_monotonic_transmit_timestamp =
        monotonic_clock::max_time;
  }

  state_[node_index_].boot_uuid = source_node_boot_uuid;

  VLOG(1) << "Rebooted " << name();
  newest_message_time_ = monotonic_clock::min_time;
  // When a node reboots, parts_uuid changes but the same writer continues to
  // write the data, so we can reset the max out of order duration. If we don't
  // do this, the max out of order duration can grow to an unreasonable value.
  max_out_of_order_duration_ = log_namer_->base_max_out_of_order_duration();
}

void NewDataWriter::UpdateBoot(const UUID &source_node_boot_uuid) {
  if (state_[node_index_].boot_uuid != source_node_boot_uuid) {
    state_[node_index_].boot_uuid = source_node_boot_uuid;
    if (header_written_) {
      Reboot(source_node_boot_uuid);
    }
  }
}

void NewDataWriter::UpdateRemote(
    const size_t remote_node_index, const UUID &remote_node_boot_uuid,
    const monotonic_clock::time_point monotonic_remote_time,
    const monotonic_clock::time_point monotonic_remote_transmit_time,
    const monotonic_clock::time_point monotonic_event_time, const bool reliable,
    monotonic_clock::time_point monotonic_timestamp_time) {
  // Trigger rotation if anything in the header changes.
  bool rotate = false;
  CHECK_LT(remote_node_index, state_.size());
  State &state = state_[remote_node_index];

  // Did the remote boot UUID change?
  if (state.boot_uuid != remote_node_boot_uuid) {
    VLOG(1) << name() << " Remote " << remote_node_index << " updated to "
            << remote_node_boot_uuid << " from " << state.boot_uuid;
    state.boot_uuid = remote_node_boot_uuid;
    state.oldest_remote_monotonic_timestamp = monotonic_clock::max_time;
    state.oldest_local_monotonic_timestamp = monotonic_clock::max_time;
    state.oldest_remote_unreliable_monotonic_timestamp =
        monotonic_clock::max_time;
    state.oldest_local_unreliable_monotonic_timestamp =
        monotonic_clock::max_time;
    state.oldest_remote_reliable_monotonic_timestamp =
        monotonic_clock::max_time;
    state.oldest_local_reliable_monotonic_timestamp = monotonic_clock::max_time;
    state.oldest_logger_remote_unreliable_monotonic_timestamp =
        monotonic_clock::max_time;
    state.oldest_logger_local_unreliable_monotonic_timestamp =
        monotonic_clock::max_time;
    state.oldest_remote_reliable_monotonic_transmit_timestamp =
        monotonic_clock::max_time;
    state.oldest_local_reliable_monotonic_transmit_timestamp =
        monotonic_clock::max_time;
    rotate = true;
  }

  if (monotonic_remote_transmit_time != monotonic_clock::min_time) {
    if (state.oldest_remote_reliable_monotonic_transmit_timestamp >
        monotonic_remote_transmit_time) {
      VLOG(1) << name() << " Remote " << remote_node_index
              << " oldest_remote_reliable_monotonic_transmit_timestamp updated "
                 "from "
              << state.oldest_remote_reliable_monotonic_transmit_timestamp
              << " to " << monotonic_remote_transmit_time;
      state.oldest_remote_reliable_monotonic_transmit_timestamp =
          monotonic_remote_transmit_time;
      state.oldest_local_reliable_monotonic_transmit_timestamp =
          monotonic_event_time;
      rotate = true;
    }
  }

  // Did the unreliable timestamps change?
  if (!reliable) {
    if (state.oldest_remote_unreliable_monotonic_timestamp >
        monotonic_remote_time) {
      VLOG(1) << name() << " Remote " << remote_node_index
              << " oldest_remote_unreliable_monotonic_timestamp updated from "
              << state.oldest_remote_unreliable_monotonic_timestamp << " to "
              << monotonic_remote_time;
      state.oldest_remote_unreliable_monotonic_timestamp =
          monotonic_remote_time;
      state.oldest_local_unreliable_monotonic_timestamp = monotonic_event_time;
      rotate = true;
    }
  } else {
    if (state.oldest_remote_reliable_monotonic_timestamp >
        monotonic_remote_time) {
      VLOG(1) << name() << " Remote " << remote_node_index
              << " oldest_remote_reliable_monotonic_timestamp updated from "
              << state.oldest_remote_reliable_monotonic_timestamp << " to "
              << monotonic_remote_time;
      state.oldest_remote_reliable_monotonic_timestamp = monotonic_remote_time;
      state.oldest_local_reliable_monotonic_timestamp = monotonic_event_time;
      rotate = true;
    }
  }

  // Track the logger timestamps too.
  if (monotonic_timestamp_time != monotonic_clock::min_time) {
    State &logger_state = state_[node_index_];
    CHECK_EQ(remote_node_index, logger_node_index_);
    if (monotonic_event_time <
        logger_state.oldest_logger_remote_unreliable_monotonic_timestamp) {
      VLOG(1)
          << name() << " Remote " << node_index_
          << " oldest_logger_remote_unreliable_monotonic_timestamp updated "
             "from "
          << logger_state.oldest_logger_remote_unreliable_monotonic_timestamp
          << " to " << monotonic_event_time;
      logger_state.oldest_logger_remote_unreliable_monotonic_timestamp =
          monotonic_event_time;
      logger_state.oldest_logger_local_unreliable_monotonic_timestamp =
          monotonic_timestamp_time;

      rotate = true;
    }
  }

  // Did any of the timestamps change?
  if (state.oldest_remote_monotonic_timestamp > monotonic_remote_time) {
    VLOG(1) << name() << " Remote " << remote_node_index
            << " oldest_remote_monotonic_timestamp updated from "
            << state.oldest_remote_monotonic_timestamp << " to "
            << monotonic_remote_time;
    state.oldest_remote_monotonic_timestamp = monotonic_remote_time;
    state.oldest_local_monotonic_timestamp = monotonic_event_time;
    rotate = true;
  }

  if (rotate) {
    Rotate();
  }
}

std::chrono::nanoseconds NewDataWriter::CopyDataMessage(
    DataEncoder::Copier *coppier, const UUID &source_node_boot_uuid,
    aos::monotonic_clock::time_point now,
    aos::monotonic_clock::time_point message_time) {
  CHECK(allowed_data_types_[static_cast<size_t>(StoredDataType::DATA)])
      << ": Tried to write data on non-data writer.";
  return CopyMessage(coppier, source_node_boot_uuid, now, message_time);
}

void NewDataWriter::CopyTimestampMessage(
    DataEncoder::Copier *coppier, const UUID &source_node_boot_uuid,
    aos::monotonic_clock::time_point now,
    aos::monotonic_clock::time_point message_time) {
  CHECK(allowed_data_types_[static_cast<size_t>(StoredDataType::TIMESTAMPS)])
      << ": Tried to write timestamps on non-timestamp writer.";
  CopyMessage(coppier, source_node_boot_uuid, now, message_time);
}

void NewDataWriter::CopyRemoteTimestampMessage(
    DataEncoder::Copier *coppier, const UUID &source_node_boot_uuid,
    aos::monotonic_clock::time_point now,
    aos::monotonic_clock::time_point message_time) {
  CHECK(allowed_data_types_[static_cast<size_t>(
      StoredDataType::REMOTE_TIMESTAMPS)])
      << ": Tried to write remote timestamps on non-remote timestamp writer.";
  CopyMessage(coppier, source_node_boot_uuid, now, message_time);
}

std::chrono::nanoseconds NewDataWriter::CopyMessage(
    DataEncoder::Copier *coppier, const UUID &source_node_boot_uuid,
    aos::monotonic_clock::time_point now,
    aos::monotonic_clock::time_point message_time) {
  // Trigger a reboot if we detect the boot UUID change.
  UpdateBoot(source_node_boot_uuid);

  if (!header_written_) {
    QueueHeader(MakeHeader());
  }

  bool max_out_of_order_duration_exceeded = false;
  // Enforce max out of duration contract.
  //
  // Updates the newest message time.
  // Rotate the part file if current message is more than
  // max_out_of_order_duration behind the newest message we've logged so far.
  if (message_time > newest_message_time_) {
    newest_message_time_ = message_time;
  }

  // Don't consider messages before start up when checking for max out of order
  // duration.
  monotonic_clock::time_point monotonic_start_time =
      log_namer_->monotonic_start_time(node_index_, source_node_boot_uuid);

  if (std::chrono::nanoseconds((newest_message_time_ -
                                std::max(monotonic_start_time, message_time))) >
      max_out_of_order_duration_) {
    // If the new message is older than 2 * max_out_order_duration, doubling it
    // won't be sufficient.
    //
    // Example: newest_message_time = 10, logged_message_time = 5,
    // max_ooo_duration = 2
    //
    // In this case actual max_ooo_duration = 10 - 5 = 5, but we double the
    // existing max_ooo_duration we get 4 which is not sufficient.
    //
    // Take the max of the two values.
    max_out_of_order_duration_ =
        2 * std::max(max_out_of_order_duration_,
                     std::chrono::nanoseconds(
                         (newest_message_time_ - message_time)));
    max_out_of_order_duration_exceeded = true;
  }

  // If the start time has changed for this node, trigger a rotation.
  if ((monotonic_start_time != monotonic_start_time_) ||
      max_out_of_order_duration_exceeded) {
    // If we just received a start time now, we will rotate parts shortly. Use a
    // reasonable max out of order durationin the new header based on start time
    // information available now.
    if ((monotonic_start_time_ == monotonic_clock::min_time) &&
        (monotonic_start_time != monotonic_clock::min_time)) {
      // If we're writing current messages  but we receive an older start time,
      // we can pick a reasonable max ooo duration number for the next part.
      //
      // For example - Our current max ooo duration is 0.3 seconds. We're
      // writing messages at 20 seconds and recieve a start time of 1 second. We
      // don't need max ooo duration to be (20 - 1) = 19 seconds although that
      // would still work.
      //
      // Pick the minimum max out of duration value that satisifies the
      // requirement but bound the minimum at the base value we started with.
      max_out_of_order_duration_ =
          std::max(log_namer_->base_max_out_of_order_duration(),
                   std::min(max_out_of_order_duration_,
                            std::chrono::nanoseconds(newest_message_time_ -
                                                     monotonic_start_time)));
    }
    CHECK(header_written_);
    Rotate();
  }

  CHECK_EQ(log_namer_->monotonic_start_time(node_index_, source_node_boot_uuid),
           monotonic_start_time_);
  CHECK_EQ(state_[node_index_].boot_uuid, source_node_boot_uuid);
  CHECK(writer);
  CHECK(header_written_) << ": Attempting to write message before header to "
                         << writer->name();
  CHECK_LE(coppier->size(), max_message_size_);
  std::chrono::nanoseconds encode_duration = writer->CopyMessage(coppier, now);
  return encode_duration;
}

aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader>
NewDataWriter::MakeHeader() {
  const size_t logger_node_index = log_namer_->logger_node_index();
  const UUID &logger_node_boot_uuid = log_namer_->logger_node_boot_uuid();
  if (state_[logger_node_index].boot_uuid == UUID::Zero()) {
    VLOG(1) << name() << " Logger node is " << logger_node_index
            << " and uuid is " << logger_node_boot_uuid;
    state_[logger_node_index].boot_uuid = logger_node_boot_uuid;
  } else {
    CHECK_EQ(state_[logger_node_index].boot_uuid, logger_node_boot_uuid);
  }
  return log_namer_->MakeHeader(node_index_, state_, parts_uuid(), parts_index_,
                                max_out_of_order_duration_,
                                allowed_data_types_);
}

void NewDataWriter::QueueHeader(
    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> &&header) {
  CHECK(!header_written_) << ": Attempting to write duplicate header to "
                          << writer->name();
  CHECK(header.message().has_source_node_boot_uuid());
  CHECK_EQ(state_[node_index_].boot_uuid,
           UUID::FromString(header.message().source_node_boot_uuid()));
  if (!writer) {
    // Since we haven't opened the first time, it's still not too late to update
    // the max message size.  Make sure the header fits.
    //
    // This won't work well on reboots, but the structure of the header is fixed
    // by that point in time, so it's size is fixed too.
    //
    // Most of the time, the minimum buffer size inside the encoder of around
    // 128k will make this a non-issue.
    UpdateMaxMessageSize(header.span().size());

    reopen_(this);
  }

  VLOG(1) << "Writing to " << name() << " "
          << aos::FlatbufferToJson(
                 header, {.multi_line = false, .max_vector_size = 100});

  CHECK(writer);
  DataEncoder::SpanCopier coppier(header.span());
  CHECK_LE(coppier.size(), max_message_size_);
  writer->CopyMessage(&coppier, aos::monotonic_clock::now());
  header_written_ = true;
  monotonic_start_time_ = log_namer_->monotonic_start_time(
      node_index_, state_[node_index_].boot_uuid);
}

void NewDataWriter::Close() {
  CHECK(writer);
  close_(this);
  writer.reset();
  header_written_ = false;
}

LogNamer::NodeState *LogNamer::GetNodeState(size_t node_index,
                                            const UUID &boot_uuid) {
  auto it = node_states_.find(std::make_pair(node_index, boot_uuid));
  if (it == node_states_.end()) {
    it =
        node_states_.emplace(std::make_pair(node_index, boot_uuid), NodeState())
            .first;
  }
  return &it->second;
}

aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> LogNamer::MakeHeader(
    size_t node_index, const std::vector<NewDataWriter::State> &state,
    const UUID &parts_uuid, int parts_index,
    std::chrono::nanoseconds max_out_of_order_duration,
    const std::array<bool, static_cast<size_t>(StoredDataType::MAX) + 1>
        &allowed_data_types) {
  const UUID &source_node_boot_uuid = state[node_index].boot_uuid;
  const Node *const source_node =
      configuration::GetNode(configuration_, node_index);
  CHECK_EQ(LogFileHeader::MiniReflectTypeTable()->num_elems, 37u)
      << ": If you added new fields to the LogFileHeader table, don't forget "
         "to add it below!";
  ;
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  flatbuffers::Offset<flatbuffers::String> config_sha256_offset;
  flatbuffers::Offset<aos::Configuration> configuration_offset;
  if (header_.message().has_configuration()) {
    CHECK(!header_.message().has_configuration_sha256());
    configuration_offset =
        CopyFlatBuffer(header_.message().configuration(), &fbb);
  } else {
    CHECK(!header_.message().has_configuration());
    CHECK(header_.message().has_configuration_sha256());
    config_sha256_offset = fbb.CreateString(
        header_.message().configuration_sha256()->string_view());
  }

  CHECK(header_.message().has_name());
  const flatbuffers::Offset<flatbuffers::String> name_offset =
      fbb.CreateString(header_.message().name()->string_view());
  const flatbuffers::Offset<flatbuffers::String> logger_sha1_offset =
      header_.message().has_logger_sha1()
          ? fbb.CreateString(header_.message().logger_sha1()->string_view())
          : 0;
  const flatbuffers::Offset<flatbuffers::String> logger_version_offset =
      header_.message().has_logger_version()
          ? fbb.CreateString(header_.message().logger_version()->string_view())
          : 0;

  CHECK(header_.message().has_log_event_uuid());
  const flatbuffers::Offset<flatbuffers::String> log_event_uuid_offset =
      fbb.CreateString(header_.message().log_event_uuid()->string_view());

  CHECK(header_.message().has_logger_instance_uuid());
  const flatbuffers::Offset<flatbuffers::String> logger_instance_uuid_offset =
      fbb.CreateString(header_.message().logger_instance_uuid()->string_view());

  flatbuffers::Offset<flatbuffers::String> log_start_uuid_offset;
  if (header_.message().has_log_start_uuid()) {
    log_start_uuid_offset =
        fbb.CreateString(header_.message().log_start_uuid()->string_view());
  }

  CHECK(header_.message().has_logger_node_boot_uuid());
  const flatbuffers::Offset<flatbuffers::String> logger_node_boot_uuid_offset =
      fbb.CreateString(
          header_.message().logger_node_boot_uuid()->string_view());

  CHECK_NE(source_node_boot_uuid, UUID::Zero());
  const flatbuffers::Offset<flatbuffers::String> source_node_boot_uuid_offset =
      source_node_boot_uuid.PackString(&fbb);

  const flatbuffers::Offset<flatbuffers::String> parts_uuid_offset =
      parts_uuid.PackString(&fbb);

  flatbuffers::Offset<Node> node_offset;
  flatbuffers::Offset<Node> logger_node_offset;

  if (configuration::MultiNode(configuration_)) {
    node_offset = RecursiveCopyFlatBuffer(source_node, &fbb);
    logger_node_offset = RecursiveCopyFlatBuffer(node_, &fbb);
  }

  std::vector<flatbuffers::Offset<flatbuffers::String>> boot_uuid_offsets;
  boot_uuid_offsets.reserve(state.size());

  int64_t *unused;
  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_remote_monotonic_timestamps_offset =
          fbb.CreateUninitializedVector(state.size(), &unused);

  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_local_monotonic_timestamps_offset =
          fbb.CreateUninitializedVector(state.size(), &unused);

  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_remote_unreliable_monotonic_timestamps_offset =
          fbb.CreateUninitializedVector(state.size(), &unused);

  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_local_unreliable_monotonic_timestamps_offset =
          fbb.CreateUninitializedVector(state.size(), &unused);

  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_remote_reliable_monotonic_timestamps_offset =
          fbb.CreateUninitializedVector(state.size(), &unused);

  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_local_reliable_monotonic_timestamps_offset =
          fbb.CreateUninitializedVector(state.size(), &unused);

  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_logger_remote_unreliable_monotonic_timestamps_offset =
          fbb.CreateUninitializedVector(state.size(), &unused);

  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_logger_local_unreliable_monotonic_timestamps_offset =
          fbb.CreateUninitializedVector(state.size(), &unused);

  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_remote_reliable_monotonic_transmit_timestamps_offset =
          fbb.CreateUninitializedVector(state.size(), &unused);

  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_local_reliable_monotonic_transmit_timestamps_offset =
          fbb.CreateUninitializedVector(state.size(), &unused);

  for (size_t i = 0; i < state.size(); ++i) {
    if (state[i].boot_uuid != UUID::Zero()) {
      boot_uuid_offsets.emplace_back(state[i].boot_uuid.PackString(&fbb));
    } else {
      boot_uuid_offsets.emplace_back(fbb.CreateString(""));
    }
    if (state[i].boot_uuid == UUID::Zero()) {
      CHECK_EQ(state[i].oldest_remote_monotonic_timestamp,
               monotonic_clock::max_time);
      CHECK_EQ(state[i].oldest_local_monotonic_timestamp,
               monotonic_clock::max_time);
      CHECK_EQ(state[i].oldest_remote_unreliable_monotonic_timestamp,
               monotonic_clock::max_time);
      CHECK_EQ(state[i].oldest_local_unreliable_monotonic_timestamp,
               monotonic_clock::max_time);
      CHECK_EQ(state[i].oldest_remote_reliable_monotonic_timestamp,
               monotonic_clock::max_time);
      CHECK_EQ(state[i].oldest_local_reliable_monotonic_timestamp,
               monotonic_clock::max_time);
      CHECK_EQ(state[i].oldest_logger_remote_unreliable_monotonic_timestamp,
               monotonic_clock::max_time);
      CHECK_EQ(state[i].oldest_logger_local_unreliable_monotonic_timestamp,
               monotonic_clock::max_time);
      CHECK_EQ(state[i].oldest_remote_reliable_monotonic_transmit_timestamp,
               monotonic_clock::max_time);
      CHECK_EQ(state[i].oldest_local_reliable_monotonic_transmit_timestamp,
               monotonic_clock::max_time);
    }

    flatbuffers::GetMutableTemporaryPointer(
        fbb, oldest_remote_monotonic_timestamps_offset)
        ->Mutate(i, state[i]
                        .oldest_remote_monotonic_timestamp.time_since_epoch()
                        .count());
    flatbuffers::GetMutableTemporaryPointer(
        fbb, oldest_local_monotonic_timestamps_offset)
        ->Mutate(i, state[i]
                        .oldest_local_monotonic_timestamp.time_since_epoch()
                        .count());
    flatbuffers::GetMutableTemporaryPointer(
        fbb, oldest_remote_unreliable_monotonic_timestamps_offset)
        ->Mutate(i, state[i]
                        .oldest_remote_unreliable_monotonic_timestamp
                        .time_since_epoch()
                        .count());
    flatbuffers::GetMutableTemporaryPointer(
        fbb, oldest_local_unreliable_monotonic_timestamps_offset)
        ->Mutate(i, state[i]
                        .oldest_local_unreliable_monotonic_timestamp
                        .time_since_epoch()
                        .count());

    flatbuffers::GetMutableTemporaryPointer(
        fbb, oldest_remote_reliable_monotonic_timestamps_offset)
        ->Mutate(i, state[i]
                        .oldest_remote_reliable_monotonic_timestamp
                        .time_since_epoch()
                        .count());
    flatbuffers::GetMutableTemporaryPointer(
        fbb, oldest_local_reliable_monotonic_timestamps_offset)
        ->Mutate(
            i, state[i]
                   .oldest_local_reliable_monotonic_timestamp.time_since_epoch()
                   .count());

    flatbuffers::GetMutableTemporaryPointer(
        fbb, oldest_logger_remote_unreliable_monotonic_timestamps_offset)
        ->Mutate(i, state[i]
                        .oldest_logger_remote_unreliable_monotonic_timestamp
                        .time_since_epoch()
                        .count());
    flatbuffers::GetMutableTemporaryPointer(
        fbb, oldest_logger_local_unreliable_monotonic_timestamps_offset)
        ->Mutate(i, state[i]
                        .oldest_logger_local_unreliable_monotonic_timestamp
                        .time_since_epoch()
                        .count());

    flatbuffers::GetMutableTemporaryPointer(
        fbb, oldest_remote_reliable_monotonic_transmit_timestamps_offset)
        ->Mutate(i, state[i]
                        .oldest_remote_reliable_monotonic_transmit_timestamp
                        .time_since_epoch()
                        .count());
    flatbuffers::GetMutableTemporaryPointer(
        fbb, oldest_local_reliable_monotonic_transmit_timestamps_offset)
        ->Mutate(i, state[i]
                        .oldest_local_reliable_monotonic_transmit_timestamp
                        .time_since_epoch()
                        .count());
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
      boot_uuids_offset = fbb.CreateVector(boot_uuid_offsets);

  aos::ErrorList<StoredDataType> allowed_data_types_vector;
  for (size_t type = static_cast<size_t>(StoredDataType::MIN);
       type <= static_cast<size_t>(StoredDataType::MAX); ++type) {
    if (allowed_data_types[type]) {
      allowed_data_types_vector.Set(static_cast<StoredDataType>(type));
    }
  }

  flatbuffers::Offset<flatbuffers::Vector<StoredDataType>> data_stored_offset =
      fbb.CreateVector(allowed_data_types_vector.data(),
                       allowed_data_types_vector.size());

  aos::logger::LogFileHeader::Builder log_file_header_builder(fbb);

  log_file_header_builder.add_name(name_offset);
  if (!logger_sha1_offset.IsNull()) {
    log_file_header_builder.add_logger_sha1(logger_sha1_offset);
  }
  if (!logger_version_offset.IsNull()) {
    log_file_header_builder.add_logger_version(logger_version_offset);
  }

  // Only add the node if we are running in a multinode configuration.
  if (!logger_node_offset.IsNull()) {
    log_file_header_builder.add_node(node_offset);
    log_file_header_builder.add_logger_node(logger_node_offset);
  }

  if (!configuration_offset.IsNull()) {
    log_file_header_builder.add_configuration(configuration_offset);
  }

  log_file_header_builder.add_max_out_of_order_duration(
      max_out_of_order_duration.count());

  NodeState *node_state = GetNodeState(node_index, source_node_boot_uuid);
  log_file_header_builder.add_monotonic_start_time(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          node_state->monotonic_start_time.time_since_epoch())
          .count());
  if (source_node == node_) {
    log_file_header_builder.add_realtime_start_time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            node_state->realtime_start_time.time_since_epoch())
            .count());
  } else {
    // Fill out the legacy start times.  Since these were implemented to never
    // change on reboot, they aren't very helpful in tracking what happened.
    log_file_header_builder.add_logger_monotonic_start_time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            node_state->logger_monotonic_start_time.time_since_epoch())
            .count());
    log_file_header_builder.add_logger_realtime_start_time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            node_state->logger_realtime_start_time.time_since_epoch())
            .count());
  }

  // TODO(austin): Add more useful times.  When was this part started?  What do
  // we know about both the logger and remote then?

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
  log_file_header_builder.add_parts_index(parts_index);

  if (!config_sha256_offset.IsNull()) {
    log_file_header_builder.add_configuration_sha256(config_sha256_offset);
  }

  log_file_header_builder.add_boot_uuids(boot_uuids_offset);
  log_file_header_builder.add_logger_part_monotonic_start_time(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          event_loop_->monotonic_now().time_since_epoch())
          .count());
  log_file_header_builder.add_logger_part_realtime_start_time(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          event_loop_->realtime_now().time_since_epoch())
          .count());
  log_file_header_builder.add_oldest_remote_monotonic_timestamps(
      oldest_remote_monotonic_timestamps_offset);
  log_file_header_builder.add_oldest_local_monotonic_timestamps(
      oldest_local_monotonic_timestamps_offset);
  log_file_header_builder.add_oldest_remote_unreliable_monotonic_timestamps(
      oldest_remote_unreliable_monotonic_timestamps_offset);
  log_file_header_builder.add_oldest_local_unreliable_monotonic_timestamps(
      oldest_local_unreliable_monotonic_timestamps_offset);
  log_file_header_builder.add_oldest_remote_reliable_monotonic_timestamps(
      oldest_remote_reliable_monotonic_timestamps_offset);
  log_file_header_builder.add_oldest_local_reliable_monotonic_timestamps(
      oldest_local_reliable_monotonic_timestamps_offset);
  log_file_header_builder
      .add_oldest_logger_remote_unreliable_monotonic_timestamps(
          oldest_logger_remote_unreliable_monotonic_timestamps_offset);
  log_file_header_builder
      .add_oldest_logger_local_unreliable_monotonic_timestamps(
          oldest_logger_local_unreliable_monotonic_timestamps_offset);
  log_file_header_builder
      .add_oldest_remote_reliable_monotonic_transmit_timestamps(
          oldest_remote_reliable_monotonic_transmit_timestamps_offset);
  log_file_header_builder
      .add_oldest_local_reliable_monotonic_transmit_timestamps(
          oldest_local_reliable_monotonic_transmit_timestamps_offset);

  log_file_header_builder.add_data_stored(data_stored_offset);
  fbb.FinishSizePrefixed(log_file_header_builder.Finish());
  aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> result(
      fbb.Release());

  CHECK(result.Verify()) << ": Built a corrupted header.";

  return result;
}

MultiNodeLogNamer::MultiNodeLogNamer(std::unique_ptr<LogBackend> log_backend,
                                     EventLoop *event_loop)
    : MultiNodeLogNamer(std::move(log_backend), event_loop->configuration(),
                        event_loop, event_loop->node()) {}

MultiNodeLogNamer::MultiNodeLogNamer(std::unique_ptr<LogBackend> log_backend,
                                     const Configuration *configuration,
                                     EventLoop *event_loop, const Node *node)
    : LogNamer(configuration, event_loop, node),
      log_backend_(std::move(log_backend)),
      encoder_factory_([](size_t max_message_size) {
        // TODO(austin): For slow channels, can we allocate less memory?
        return std::make_unique<DummyEncoder>(max_message_size,
                                              absl::GetFlag(FLAGS_flush_size));
      }) {}

MultiNodeLogNamer::~MultiNodeLogNamer() {
  if (!ran_out_of_space_) {
    // This handles renaming temporary files etc.
    Close();
  }
}

void MultiNodeLogNamer::Rotate(const Node *node) {
  for (auto &data_map : {&node_data_writers_, &node_timestamp_writers_}) {
    auto it = data_map->find(node);
    if (it != data_map->end()) {
      it->second.Rotate();
    }
  }
}

void MultiNodeLogNamer::WriteConfiguration(
    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
    std::string_view config_sha256) {
  if (ran_out_of_space_) {
    return;
  }

  const std::string filename = absl::StrCat(config_sha256, ".bfbs", extension_);
  auto file_handle = log_backend_->RequestFile(filename);
  std::unique_ptr<DetachedBufferWriter> writer =
      std::make_unique<DetachedBufferWriter>(
          std::move(file_handle), encoder_factory_(header->span().size()));

  DataEncoder::SpanCopier coppier(header->span());
  writer->CopyMessage(&coppier, aos::monotonic_clock::now());

  if (!writer->ran_out_of_space()) {
    all_filenames_.emplace_back(filename);
  }
  // Close the file and maybe rename it too.
  CloseWriter(&writer);
}

void MultiNodeLogNamer::NoticeNode(const Node *source_node) {
  if (std::find(nodes_.begin(), nodes_.end(), source_node) == nodes_.end()) {
    nodes_.emplace_back(source_node);
  }
}

NewDataWriter *MultiNodeLogNamer::FindNodeDataWriter(const Node *source_node,
                                                     size_t max_message_size) {
  NoticeNode(source_node);

  auto it = node_data_writers_.find(source_node);
  if (it != node_data_writers_.end()) {
    it->second.UpdateMaxMessageSize(max_message_size);
    return &(it->second);
  }
  return nullptr;
}

NewDataWriter *MultiNodeLogNamer::FindNodeTimestampWriter(
    const Node *source_node, size_t max_message_size) {
  NoticeNode(source_node);

  auto it = node_timestamp_writers_.find(source_node);
  if (it != node_timestamp_writers_.end()) {
    it->second.UpdateMaxMessageSize(max_message_size);
    return &(it->second);
  }
  return nullptr;
}

NewDataWriter *MultiNodeLogNamer::AddNodeDataWriter(const Node *source_node,
                                                    NewDataWriter &&writer) {
  auto result = node_data_writers_.emplace(source_node, std::move(writer));
  CHECK(result.second);
  return &(result.first->second);
}

NewDataWriter *MultiNodeLogNamer::AddNodeTimestampWriter(
    const Node *source_node, NewDataWriter &&writer) {
  auto result = node_timestamp_writers_.emplace(source_node, std::move(writer));
  CHECK(result.second);
  return &(result.first->second);
}

NewDataWriter *MultiNodeLogNamer::MakeWriter(const Channel *channel) {
  // See if we can read the data on this node at all.
  const bool is_readable =
      configuration::ChannelIsReadableOnNode(channel, this->node());
  if (!is_readable) {
    return nullptr;
  }

  // Then, see if we are supposed to log the data here.
  const bool log_message =
      configuration::ChannelMessageIsLoggedOnNode(channel, this->node());

  if (!log_message) {
    return nullptr;
  }

  // Ok, we have data that is being forwarded to us that we are supposed to
  // log.  It needs to be logged with send timestamps, but be sorted enough
  // to be able to be processed.

  const Node *source_node =
      configuration::MultiNode(configuration_)
          ? configuration::GetNode(configuration_,
                                   channel->source_node()->string_view())
          : nullptr;

  // If we already have a data writer for the node, then use the same writer for
  // all channels of that node.
  NewDataWriter *result = FindNodeDataWriter(
      source_node,
      PackMessageSize(LogType::kLogRemoteMessage, channel->max_size()));
  if (result != nullptr) {
    return result;
  }

  // If we don't have a data writer for the node, create one.
  return AddNodeDataWriter(
      source_node,
      NewDataWriter{
          this,
          source_node,
          node_,
          [this, source_node](NewDataWriter *data_writer) {
            OpenDataWriter(source_node, data_writer);
          },
          [this](NewDataWriter *data_writer) {
            CloseWriter(&data_writer->writer);
          },
          PackMessageSize(LogType::kLogRemoteMessage, channel->max_size()),
          {StoredDataType::DATA}});
}

NewDataWriter *MultiNodeLogNamer::MakeForwardedTimestampWriter(
    const Channel *channel, const Node *node) {
  // See if we can read the data on this node at all.
  const bool is_readable =
      configuration::ChannelIsReadableOnNode(channel, this->node());
  CHECK(is_readable) << ": " << configuration::CleanedChannelToString(channel);

  CHECK_NE(node, this->node());

  // If we have a remote timestamp writer for a particular node, use the same
  // writer for all remote timestamp channels of that node.
  NewDataWriter *result =
      FindNodeTimestampWriter(node, PackRemoteMessageSize());
  if (result != nullptr) {
    return result;
  }

  // If there are no remote timestamp writers for the node, create one.
  return AddNodeTimestampWriter(
      node, NewDataWriter{this,
                          configuration::GetNode(configuration_, node),
                          node_,
                          [this](NewDataWriter *data_writer) {
                            OpenForwardedTimestampWriter(node_, data_writer);
                          },
                          [this](NewDataWriter *data_writer) {
                            CloseWriter(&data_writer->writer);
                          },
                          PackRemoteMessageSize(),
                          {StoredDataType::REMOTE_TIMESTAMPS}});
}

NewDataWriter *MultiNodeLogNamer::MakeTimestampWriter(const Channel *channel) {
  bool log_delivery_times = false;
  if (this->node() != nullptr) {
    log_delivery_times = configuration::ConnectionDeliveryTimeIsLoggedOnNode(
        channel, this->node(), this->node());
  }
  if (!log_delivery_times) {
    return nullptr;
  }

  // There is only one of these.
  NewDataWriter *result = FindNodeTimestampWriter(
      this->node(), PackMessageSize(LogType::kLogDeliveryTimeOnly, 0));
  if (result != nullptr) {
    return result;
  }

  return AddNodeTimestampWriter(
      node_, NewDataWriter{this,
                           node_,
                           node_,
                           [this](NewDataWriter *data_writer) {
                             OpenTimestampWriter(data_writer);
                           },
                           [this](NewDataWriter *data_writer) {
                             CloseWriter(&data_writer->writer);
                           },
                           PackMessageSize(LogType::kLogDeliveryTimeOnly, 0),
                           {StoredDataType::TIMESTAMPS}});
}

WriteCode MultiNodeLogNamer::Close() {
  node_data_writers_.clear();
  node_timestamp_writers_.clear();
  if (ran_out_of_space_) {
    return WriteCode::kOutOfSpace;
  }
  return WriteCode::kOk;
}

void MultiNodeLogNamer::ResetStatistics() {
  for (std::pair<const Node *const, NewDataWriter> &data_writer :
       node_data_writers_) {
    if (!data_writer.second.writer) continue;
    data_writer.second.writer->WriteStatistics()->ResetStats();
  }
  for (std::pair<const Node *const, NewDataWriter> &data_writer :
       node_timestamp_writers_) {
    if (!data_writer.second.writer) continue;
    data_writer.second.writer->WriteStatistics()->ResetStats();
  }
  max_write_time_ = std::chrono::nanoseconds::zero();
  max_write_time_bytes_ = -1;
  max_write_time_messages_ = -1;
  total_write_time_ = std::chrono::nanoseconds::zero();
  total_write_count_ = 0;
  total_write_messages_ = 0;
  total_write_bytes_ = 0;
}

void MultiNodeLogNamer::OpenForwardedTimestampWriter(
    const Node * /*source_node*/, NewDataWriter *data_writer) {
  const std::string filename = absl::StrCat(
      "timestamps/remote_", data_writer->node()->name()->string_view(), ".part",
      data_writer->parts_index(), ".bfbs", extension_);
  CreateBufferWriter(filename, data_writer->max_message_size(),
                     &data_writer->writer);
}

void MultiNodeLogNamer::OpenDataWriter(const Node *source_node,
                                       NewDataWriter *data_writer) {
  std::string filename;

  if (source_node != nullptr) {
    if (source_node == node_) {
      filename = absl::StrCat(source_node->name()->string_view(), "_");
    } else {
      filename = absl::StrCat("data/", source_node->name()->string_view(), "_");
    }
  }

  absl::StrAppend(&filename, "data.part", data_writer->parts_index(), ".bfbs",
                  extension_);
  CreateBufferWriter(filename, data_writer->max_message_size(),
                     &data_writer->writer);
}

void MultiNodeLogNamer::OpenTimestampWriter(NewDataWriter *data_writer) {
  std::string filename =
      absl::StrCat(node()->name()->string_view(), "_timestamps.part",
                   data_writer->parts_index(), ".bfbs", extension_);
  CreateBufferWriter(filename, data_writer->max_message_size(),
                     &data_writer->writer);
}

void MultiNodeLogNamer::CreateBufferWriter(
    std::string_view path, size_t max_message_size,
    std::unique_ptr<DetachedBufferWriter> *destination) {
  if (ran_out_of_space_) {
    // Refuse to open any new files, which might skip data. Any existing files
    // are in the same folder, which means they're on the same filesystem, which
    // means they're probably going to run out of space and get stuck too.
    if (!(*destination)) {
      // But avoid leaving a nullptr writer if we're out of space when
      // attempting to open the first file.
      *destination = std::make_unique<DetachedBufferWriter>(
          DetachedBufferWriter::already_out_of_space_t());
    }
    return;
  }

  // Let's check that we need to close and replace current driver.
  if (*destination) {
    // Let's close the current writer.
    CloseWriter(destination);
    // Are we out of space now?
    if (ran_out_of_space_) {
      *destination = std::make_unique<DetachedBufferWriter>(
          DetachedBufferWriter::already_out_of_space_t());
      return;
    }
  }

  const std::string filename(path);
  *destination = std::make_unique<DetachedBufferWriter>(
      log_backend_->RequestFile(filename), encoder_factory_(max_message_size));
  if (!(*destination)->ran_out_of_space()) {
    all_filenames_.emplace_back(path);
  }
}

void MultiNodeLogNamer::CloseWriter(
    std::unique_ptr<DetachedBufferWriter> *writer_pointer) {
  CHECK(writer_pointer != nullptr);
  if (!(*writer_pointer)) {
    return;
  }
  DetachedBufferWriter *const writer = writer_pointer->get();
  writer->Close();

  const auto *stats = writer->WriteStatistics();
  if (stats->max_write_time() > max_write_time_) {
    max_write_time_ = stats->max_write_time();
    max_write_time_bytes_ = stats->max_write_time_bytes();
    max_write_time_messages_ = stats->max_write_time_messages();
  }
  total_write_time_ += stats->total_write_time();
  total_write_count_ += stats->total_write_count();
  total_write_messages_ += stats->total_write_messages();
  total_write_bytes_ += stats->total_write_bytes();

  if (writer->ran_out_of_space()) {
    ran_out_of_space_ = true;
    writer->acknowledge_out_of_space();
  }
}

NewDataWriter *MinimalFileMultiNodeLogNamer::MakeWriter(
    const Channel *channel) {
  // See if we can read the data on this node at all.
  const bool is_readable =
      configuration::ChannelIsReadableOnNode(channel, this->node());
  if (!is_readable) {
    return nullptr;
  }

  // Then, see if we are supposed to log the data here.
  const bool log_message =
      configuration::ChannelMessageIsLoggedOnNode(channel, this->node());

  if (!log_message) {
    return nullptr;
  }

  // Ok, we have data that is being forwarded to us that we are supposed to
  // log.  It needs to be logged with send timestamps, but be sorted enough
  // to be able to be processed.

  const Node *source_node =
      configuration::MultiNode(configuration_)
          ? configuration::GetNode(configuration_,
                                   channel->source_node()->string_view())
          : nullptr;

  // If we don't have a data writer for the node, create one.
  if (this->node() == source_node) {
    // If we already have a data writer for the node, then use the same writer
    // for all channels of that node.
    NewDataWriter *result = FindNodeDataWriter(
        source_node,
        PackMessageSize(LogType::kLogMessage, channel->max_size()));
    if (result != nullptr) {
      return result;
    }

    return AddNodeDataWriter(
        source_node,
        NewDataWriter{
            this,
            source_node,
            node_,
            [this, source_node](NewDataWriter *data_writer) {
              OpenNodeWriter(source_node, data_writer);
            },
            [this](NewDataWriter *data_writer) {
              CloseWriter(&data_writer->writer);
            },
            PackMessageSize(LogType::kLogMessage, channel->max_size()),
            {StoredDataType::DATA, StoredDataType::TIMESTAMPS}});
  } else {
    // If we already have a data writer for the node, then use the same writer
    // for all channels of that node.
    NewDataWriter *result = FindNodeDataWriter(
        source_node,
        PackMessageSize(LogType::kLogRemoteMessage, channel->max_size()));
    if (result != nullptr) {
      return result;
    }

    return AddNodeDataWriter(
        source_node,
        NewDataWriter{
            this,
            source_node,
            node_,
            [this, source_node](NewDataWriter *data_writer) {
              OpenNodeWriter(source_node, data_writer);
            },
            [this](NewDataWriter *data_writer) {
              CloseWriter(&data_writer->writer);
            },
            PackMessageSize(LogType::kLogRemoteMessage, channel->max_size()),
            {StoredDataType::DATA, StoredDataType::REMOTE_TIMESTAMPS}});
  }
}

NewDataWriter *MinimalFileMultiNodeLogNamer::MakeTimestampWriter(
    const Channel *channel) {
  bool log_delivery_times = false;
  if (this->node() != nullptr) {
    log_delivery_times = configuration::ConnectionDeliveryTimeIsLoggedOnNode(
        channel, this->node(), this->node());
  }
  if (!log_delivery_times) {
    return nullptr;
  }

  // There is only one of these.
  NewDataWriter *result = FindNodeDataWriter(
      this->node(), PackMessageSize(LogType::kLogDeliveryTimeOnly, 0));
  if (result != nullptr) {
    return result;
  }

  return AddNodeDataWriter(
      node_, NewDataWriter{this,
                           node_,
                           node_,
                           [this](NewDataWriter *data_writer) {
                             OpenNodeWriter(node_, data_writer);
                           },
                           [this](NewDataWriter *data_writer) {
                             CloseWriter(&data_writer->writer);
                           },
                           PackMessageSize(LogType::kLogDeliveryTimeOnly, 0),
                           {StoredDataType::DATA, StoredDataType::TIMESTAMPS}});
}

NewDataWriter *MinimalFileMultiNodeLogNamer::MakeForwardedTimestampWriter(
    const Channel *channel, const Node *node) {
  // See if we can read the data on this node at all.
  const bool is_readable =
      configuration::ChannelIsReadableOnNode(channel, this->node());
  CHECK(is_readable) << ": " << configuration::CleanedChannelToString(channel);

  CHECK_NE(node, this->node());

  // If we have a remote timestamp writer for a particular node, use the same
  // writer for all remote timestamp channels of that node.
  NewDataWriter *result = FindNodeDataWriter(node, PackRemoteMessageSize());
  if (result != nullptr) {
    return result;
  }

  // If there are no remote timestamp writers for the node, create one.
  return AddNodeDataWriter(
      node,
      NewDataWriter{this,
                    configuration::GetNode(configuration_, node),
                    node_,
                    [this, node](NewDataWriter *data_writer) {
                      OpenNodeWriter(node, data_writer);
                    },
                    [this](NewDataWriter *data_writer) {
                      CloseWriter(&data_writer->writer);
                    },
                    PackRemoteMessageSize(),
                    {StoredDataType::DATA, StoredDataType::REMOTE_TIMESTAMPS}});
}

void MinimalFileMultiNodeLogNamer::OpenNodeWriter(const Node *source_node,
                                                  NewDataWriter *data_writer) {
  std::string filename;

  if (node() != nullptr) {
    filename = absl::StrCat(node()->name()->string_view(), "_");
  }

  if (source_node != nullptr) {
    absl::StrAppend(&filename, source_node->name()->string_view(), "_");
  }

  absl::StrAppend(&filename, "all.part", data_writer->parts_index(), ".bfbs",
                  extension_);
  VLOG(1) << "Going to open " << filename;
  CreateBufferWriter(filename, data_writer->max_message_size(),
                     &data_writer->writer);
}

}  // namespace aos::logger
