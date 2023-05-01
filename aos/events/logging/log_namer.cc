#include "aos/events/logging/log_namer.h"

#include <functional>
#include <map>
#include <memory>
#include <string_view>
#include <vector>

#include "absl/strings/str_cat.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/flatbuffer_merge.h"
#include "aos/uuid.h"
#include "flatbuffers/flatbuffers.h"
#include "glog/logging.h"

DECLARE_int32(flush_size);

namespace aos {
namespace logger {

NewDataWriter::NewDataWriter(LogNamer *log_namer, const Node *node,
                             const Node *logger_node,
                             std::function<void(NewDataWriter *)> reopen,
                             std::function<void(NewDataWriter *)> close,
                             size_t max_message_size)
    : node_(node),
      node_index_(configuration::GetNodeIndex(log_namer->configuration_, node)),
      logger_node_index_(
          configuration::GetNodeIndex(log_namer->configuration_, logger_node)),
      log_namer_(log_namer),
      reopen_(std::move(reopen)),
      close_(std::move(close)),
      max_message_size_(max_message_size) {
  state_.resize(configuration::NodesCount(log_namer->configuration_));
  CHECK_LT(node_index_, state_.size());
}

NewDataWriter::~NewDataWriter() {
  if (writer) {
    Close();
  }
}

void NewDataWriter::Rotate() {
  // No need to rotate if nothing has been written.
  if (header_written_) {
    VLOG(1) << "Rotated " << filename();
    ++parts_index_;
    reopen_(this);
    header_written_ = false;
    QueueHeader(MakeHeader());
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
  }

  state_[node_index_].boot_uuid = source_node_boot_uuid;

  VLOG(1) << "Rebooted " << filename();
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
    const monotonic_clock::time_point monotonic_event_time, const bool reliable,
    monotonic_clock::time_point monotonic_timestamp_time) {
  // Trigger rotation if anything in the header changes.
  bool rotate = false;
  CHECK_LT(remote_node_index, state_.size());
  State &state = state_[remote_node_index];

  // Did the remote boot UUID change?
  if (state.boot_uuid != remote_node_boot_uuid) {
    VLOG(1) << filename() << " Remote " << remote_node_index << " updated to "
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
    rotate = true;
  }

  // Did the unreliable timestamps change?
  if (!reliable) {
    if (state.oldest_remote_unreliable_monotonic_timestamp >
        monotonic_remote_time) {
      VLOG(1) << filename() << " Remote " << remote_node_index
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
      VLOG(1) << filename() << " Remote " << remote_node_index
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
          << filename() << " Remote " << node_index_
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
    VLOG(1) << filename() << " Remote " << remote_node_index
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

void NewDataWriter::CopyMessage(DataEncoder::Copier *coppier,
                                const UUID &source_node_boot_uuid,
                                aos::monotonic_clock::time_point now) {
  // Trigger a reboot if we detect the boot UUID change.
  UpdateBoot(source_node_boot_uuid);

  if (!header_written_) {
    QueueHeader(MakeHeader());
  }

  // If the start time has changed for this node, trigger a rotation.
  if (log_namer_->monotonic_start_time(node_index_, source_node_boot_uuid) !=
      monotonic_start_time_) {
    CHECK(header_written_);
    Rotate();
  }

  CHECK_EQ(log_namer_->monotonic_start_time(node_index_, source_node_boot_uuid),
           monotonic_start_time_);
  CHECK_EQ(state_[node_index_].boot_uuid, source_node_boot_uuid);
  CHECK(writer);
  CHECK(header_written_) << ": Attempting to write message before header to "
                         << writer->filename();
  writer->CopyMessage(coppier, now);
}

aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader>
NewDataWriter::MakeHeader() {
  const size_t logger_node_index = log_namer_->logger_node_index();
  const UUID &logger_node_boot_uuid = log_namer_->logger_node_boot_uuid();
  if (state_[logger_node_index].boot_uuid == UUID::Zero()) {
    VLOG(1) << filename() << " Logger node is " << logger_node_index
            << " and uuid is " << logger_node_boot_uuid;
    state_[logger_node_index].boot_uuid = logger_node_boot_uuid;
  } else {
    CHECK_EQ(state_[logger_node_index].boot_uuid, logger_node_boot_uuid);
  }
  return log_namer_->MakeHeader(node_index_, state_, parts_uuid(),
                                parts_index_);
}

void NewDataWriter::QueueHeader(
    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> &&header) {
  CHECK(!header_written_) << ": Attempting to write duplicate header to "
                          << writer->filename();
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

  VLOG(1) << "Writing to " << filename() << " "
          << aos::FlatbufferToJson(
                 header, {.multi_line = false, .max_vector_size = 100});

  CHECK(writer);
  DataEncoder::SpanCopier coppier(header.span());
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
    const UUID &parts_uuid, int parts_index) {
  const UUID &source_node_boot_uuid = state[node_index].boot_uuid;
  const Node *const source_node =
      configuration::GetNode(configuration_, node_index);
  CHECK_EQ(LogFileHeader::MiniReflectTypeTable()->num_elems, 34u);
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
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
      boot_uuids_offset = fbb.CreateVector(boot_uuid_offsets);

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
      header_.message().max_out_of_order_duration());

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
  fbb.FinishSizePrefixed(log_file_header_builder.Finish());
  aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> result(
      fbb.Release());

  CHECK(result.Verify()) << ": Built a corrupted header.";

  return result;
}

MultiNodeLogNamer::MultiNodeLogNamer(
    std::unique_ptr<RenamableFileBackend> log_backend, EventLoop *event_loop)
    : MultiNodeLogNamer(std::move(log_backend), event_loop->configuration(),
                        event_loop, event_loop->node()) {}

MultiNodeLogNamer::MultiNodeLogNamer(
    std::unique_ptr<RenamableFileBackend> log_backend,
    const Configuration *configuration, EventLoop *event_loop, const Node *node)
    : LogNamer(configuration, event_loop, node),
      log_backend_(std::move(log_backend)),
      encoder_factory_([](size_t max_message_size) {
        // TODO(austin): For slow channels, can we allocate less memory?
        return std::make_unique<DummyEncoder>(max_message_size,
                                              FLAGS_flush_size);
      }) {}

MultiNodeLogNamer::~MultiNodeLogNamer() {
  if (!ran_out_of_space_) {
    // This handles renaming temporary files etc.
    Close();
  }
}

void MultiNodeLogNamer::Rotate(const Node *node) {
  if (node == this->node()) {
    if (data_writer_) {
      data_writer_->Rotate();
    }
  } else {
    for (std::pair<const Channel *const, NewDataWriter> &data_writer :
         data_writers_) {
      if (node == data_writer.second.node()) {
        data_writer.second.Rotate();
      }
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

  // Now, sort out if this is data generated on this node, or not.  It is
  // generated if it is sendable on this node.
  if (configuration::ChannelIsSendableOnNode(channel, this->node())) {
    if (!data_writer_) {
      MakeDataWriter();
    }
    data_writer_->UpdateMaxMessageSize(
        PackMessageSize(LogType::kLogRemoteMessage, channel->max_size()));
    return data_writer_.get();
  }

  // Ok, we have data that is being forwarded to us that we are supposed to
  // log.  It needs to be logged with send timestamps, but be sorted enough
  // to be able to be processed.
  CHECK(data_writers_.find(channel) == data_writers_.end());

  // Track that this node is being logged.
  const Node *source_node = configuration::GetNode(
      configuration_, channel->source_node()->string_view());

  if (std::find(nodes_.begin(), nodes_.end(), source_node) == nodes_.end()) {
    nodes_.emplace_back(source_node);
  }

  NewDataWriter data_writer(
      this, source_node, node_,
      [this, channel](NewDataWriter *data_writer) {
        OpenWriter(channel, data_writer);
      },
      [this](NewDataWriter *data_writer) { CloseWriter(&data_writer->writer); },
      PackMessageSize(LogType::kLogRemoteMessage, channel->max_size()));
  return &(
      data_writers_.emplace(channel, std::move(data_writer)).first->second);
}

NewDataWriter *MultiNodeLogNamer::MakeForwardedTimestampWriter(
    const Channel *channel, const Node *node) {
  // See if we can read the data on this node at all.
  const bool is_readable =
      configuration::ChannelIsReadableOnNode(channel, this->node());
  CHECK(is_readable) << ": " << configuration::CleanedChannelToString(channel);

  CHECK(data_writers_.find(channel) == data_writers_.end());

  if (std::find(nodes_.begin(), nodes_.end(), node) == nodes_.end()) {
    nodes_.emplace_back(node);
  }

  NewDataWriter data_writer(
      this, configuration::GetNode(configuration_, node), node_,
      [this, channel](NewDataWriter *data_writer) {
        OpenForwardedTimestampWriter(channel, data_writer);
      },
      [this](NewDataWriter *data_writer) { CloseWriter(&data_writer->writer); },
      PackRemoteMessageSize());
  return &(
      data_writers_.emplace(channel, std::move(data_writer)).first->second);
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

  if (!data_writer_) {
    MakeDataWriter();
  }
  data_writer_->UpdateMaxMessageSize(
      PackMessageSize(LogType::kLogDeliveryTimeOnly, 0));
  return data_writer_.get();
}

WriteCode MultiNodeLogNamer::Close() {
  data_writers_.clear();
  data_writer_.reset();
  if (ran_out_of_space_) {
    return WriteCode::kOutOfSpace;
  }
  return WriteCode::kOk;
}

void MultiNodeLogNamer::ResetStatistics() {
  for (std::pair<const Channel *const, NewDataWriter> &data_writer :
       data_writers_) {
    if (!data_writer.second.writer) continue;
    data_writer.second.writer->WriteStatistics()->ResetStats();
  }
  if (data_writer_) {
    data_writer_->writer->WriteStatistics()->ResetStats();
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
    const Channel *channel, NewDataWriter *data_writer) {
  std::string filename =
      absl::StrCat("timestamps", channel->name()->string_view(), "/",
                   channel->type()->string_view(), ".part",
                   data_writer->parts_index(), ".bfbs", extension_);
  CreateBufferWriter(filename, data_writer->max_message_size(),
                     &data_writer->writer);
}

void MultiNodeLogNamer::OpenWriter(const Channel *channel,
                                   NewDataWriter *data_writer) {
  const std::string filename = absl::StrCat(
      CHECK_NOTNULL(channel->source_node())->string_view(), "_data",
      channel->name()->string_view(), "/", channel->type()->string_view(),
      ".part", data_writer->parts_index(), ".bfbs", extension_);
  CreateBufferWriter(filename, data_writer->max_message_size(),
                     &data_writer->writer);
}

void MultiNodeLogNamer::MakeDataWriter() {
  if (!data_writer_) {
    data_writer_ = std::make_unique<NewDataWriter>(
        this, node_, node_,
        [this](NewDataWriter *writer) {
          std::string name;
          if (node() != nullptr) {
            name = absl::StrCat(name, node()->name()->string_view(), "_");
          }
          absl::StrAppend(&name, "data.part", writer->parts_index(), ".bfbs",
                          extension_);
          CreateBufferWriter(name, writer->max_message_size(), &writer->writer);
        },
        [this](NewDataWriter *data_writer) {
          CloseWriter(&data_writer->writer);
        },
        // Default size is 0 so it will be obvious if something doesn't fix it
        // afterwards.
        0);
  }
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
  CHECK_NOTNULL(writer_pointer);
  if (!(*writer_pointer)) {
    return;
  }
  DetachedBufferWriter *const writer = writer_pointer->get();
  const bool was_open = writer->is_open();
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

  if (!was_open) {
    CHECK(access(std::string(writer->filename()).c_str(), F_OK) == -1)
        << ": File should not exist: " << writer->filename();
  }
}

}  // namespace logger
}  // namespace aos
