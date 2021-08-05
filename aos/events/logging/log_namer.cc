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

namespace aos {
namespace logger {

NewDataWriter::NewDataWriter(LogNamer *log_namer, const Node *node,
                             std::function<void(NewDataWriter *)> reopen,
                             std::function<void(NewDataWriter *)> close)
    : node_(node),
      node_index_(configuration::GetNodeIndex(log_namer->configuration_, node)),
      log_namer_(log_namer),
      reopen_(std::move(reopen)),
      close_(std::move(close)) {
  state_.resize(configuration::NodesCount(log_namer->configuration_));
  CHECK_LT(node_index_, state_.size());
  reopen_(this);
}

NewDataWriter::~NewDataWriter() {
  if (writer) {
    Close();
  }
}

void NewDataWriter::Rotate() {
  // No need to rotate if nothing has been written.
  if (header_written_) {
    ++parts_index_;
    reopen_(this);
    header_written_ = false;
    QueueHeader(MakeHeader());
  }
}

void NewDataWriter::Reboot() {
  parts_uuid_ = UUID::Random();
  ++parts_index_;
  reopen_(this);
  header_written_ = false;
}

void NewDataWriter::UpdateRemote(
    const size_t remote_node_index, const UUID &remote_node_boot_uuid,
    const monotonic_clock::time_point monotonic_remote_time,
    const monotonic_clock::time_point monotonic_event_time,
    const bool reliable) {
  bool rotate = false;
  CHECK_LT(remote_node_index, state_.size());
  State &state = state_[remote_node_index];
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
    rotate = true;
  }

  if (!reliable) {
    if (state.oldest_remote_unreliable_monotonic_timestamp >
        monotonic_remote_time) {
      state.oldest_remote_unreliable_monotonic_timestamp =
          monotonic_remote_time;
      state.oldest_local_unreliable_monotonic_timestamp = monotonic_event_time;
      rotate = true;
    }
  }

  if (state.oldest_remote_monotonic_timestamp > monotonic_remote_time) {
    state.oldest_remote_monotonic_timestamp = monotonic_remote_time;
    state.oldest_local_monotonic_timestamp = monotonic_event_time;
    rotate = true;
  }

  if (rotate) {
    Rotate();
  }
}

void NewDataWriter::QueueMessage(flatbuffers::FlatBufferBuilder *fbb,
                                 const UUID &source_node_boot_uuid,
                                 aos::monotonic_clock::time_point now) {
  // TODO(austin): Handle remote nodes changing too, not just the source node.
  if (state_[node_index_].boot_uuid != source_node_boot_uuid) {
    state_[node_index_].boot_uuid = source_node_boot_uuid;
    if (header_written_) {
      Reboot();
    }

    QueueHeader(MakeHeader());
  }
  CHECK_EQ(state_[node_index_].boot_uuid, source_node_boot_uuid);
  CHECK(header_written_) << ": Attempting to write message before header to "
                         << writer->filename();
  writer->QueueSizedFlatbuffer(fbb, now);
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
  // TODO(austin): This triggers a dummy allocation that we don't need as part
  // of releasing.  Can we skip it?
  writer->QueueSizedFlatbuffer(header.Release());
  header_written_ = true;
}

void NewDataWriter::Close() {
  CHECK(writer);
  close_(this);
  writer.reset();
  header_written_ = false;
}

aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> LogNamer::MakeHeader(
    size_t node_index, const std::vector<NewDataWriter::State> &state,
    const UUID &parts_uuid, int parts_index) const {
  const UUID &source_node_boot_uuid = state[node_index].boot_uuid;
  const Node *const source_node =
      configuration::GetNode(configuration_, node_index);
  CHECK_EQ(LogFileHeader::MiniReflectTypeTable()->num_elems, 24u);
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
  for (const NewDataWriter::State &state : state) {
    if (state.boot_uuid != UUID::Zero()) {
      boot_uuid_offsets.emplace_back(state.boot_uuid.PackString(&fbb));
    } else {
      boot_uuid_offsets.emplace_back(fbb.CreateString(""));
    }
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
      boot_uuids_offset = fbb.CreateVector(boot_uuid_offsets);

  int64_t *oldest_remote_monotonic_timestamps;
  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_remote_monotonic_timestamps_offset = fbb.CreateUninitializedVector(
          state.size(), &oldest_remote_monotonic_timestamps);

  int64_t *oldest_local_monotonic_timestamps;
  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_local_monotonic_timestamps_offset = fbb.CreateUninitializedVector(
          state.size(), &oldest_local_monotonic_timestamps);

  int64_t *oldest_remote_unreliable_monotonic_timestamps;
  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_remote_unreliable_monotonic_timestamps_offset =
          fbb.CreateUninitializedVector(
              state.size(), &oldest_remote_unreliable_monotonic_timestamps);

  int64_t *oldest_local_unreliable_monotonic_timestamps;
  flatbuffers::Offset<flatbuffers::Vector<int64_t>>
      oldest_local_unreliable_monotonic_timestamps_offset =
          fbb.CreateUninitializedVector(
              state.size(), &oldest_local_unreliable_monotonic_timestamps);

  for (size_t i = 0; i < state.size(); ++i) {
    oldest_remote_monotonic_timestamps[i] =
        state[i].oldest_remote_monotonic_timestamp.time_since_epoch().count();
    oldest_local_monotonic_timestamps[i] =
        state[i].oldest_local_monotonic_timestamp.time_since_epoch().count();
    oldest_remote_unreliable_monotonic_timestamps[i] =
        state[i]
            .oldest_remote_unreliable_monotonic_timestamp.time_since_epoch()
            .count();
    oldest_local_unreliable_monotonic_timestamps[i] =
        state[i]
            .oldest_local_unreliable_monotonic_timestamp.time_since_epoch()
            .count();
  }

  aos::logger::LogFileHeader::Builder log_file_header_builder(fbb);

  log_file_header_builder.add_name(name_offset);

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

  log_file_header_builder.add_monotonic_start_time(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          node_states_[node_index].monotonic_start_time.time_since_epoch())
          .count());
  if (source_node == node_) {
    log_file_header_builder.add_realtime_start_time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            node_states_[node_index].realtime_start_time.time_since_epoch())
            .count());
  } else {
    // Fill out the legacy start times.  Since these were implemented to never
    // change on reboot, they aren't very helpful in tracking what happened.
    log_file_header_builder.add_logger_monotonic_start_time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            node_states_[node_index]
                .logger_monotonic_start_time.time_since_epoch())
            .count());
    log_file_header_builder.add_logger_realtime_start_time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            node_states_[node_index]
                .logger_realtime_start_time.time_since_epoch())
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
  fbb.FinishSizePrefixed(log_file_header_builder.Finish());
  aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> result(
      fbb.Release());

  CHECK(result.Verify()) << ": Built a corrupted header.";

  return result;
}

NewDataWriter *LocalLogNamer::MakeWriter(const Channel *channel) {
  CHECK(configuration::ChannelIsSendableOnNode(channel, node()))
      << ": " << configuration::CleanedChannelToString(channel);
  return &data_writer_;
}

void LocalLogNamer::Rotate(const Node *node) {
  CHECK(node == this->node());
  data_writer_.Rotate();
}

void LocalLogNamer::WriteConfiguration(
    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
    std::string_view config_sha256) {
  const std::string filename = absl::StrCat(base_name_, config_sha256, ".bfbs");

  std::unique_ptr<DetachedBufferWriter> writer =
      std::make_unique<DetachedBufferWriter>(
          filename, std::make_unique<aos::logger::DummyEncoder>());
  writer->QueueSizedFlatbuffer(header->Release());
}

NewDataWriter *LocalLogNamer::MakeTimestampWriter(const Channel *channel) {
  CHECK(configuration::ChannelIsReadableOnNode(channel, node_))
      << ": Message is not delivered to this node.";
  CHECK(node_ != nullptr) << ": Can't log timestamps in a single node world";
  CHECK(configuration::ConnectionDeliveryTimeIsLoggedOnNode(channel, node_,
                                                            node_))
      << ": Delivery times aren't logged for this channel on this node.";
  return &data_writer_;
}

NewDataWriter *LocalLogNamer::MakeForwardedTimestampWriter(
    const Channel * /*channel*/, const Node * /*node*/) {
  LOG(FATAL) << "Can't log forwarded timestamps in a singe log file.";
  return nullptr;
}

MultiNodeLogNamer::MultiNodeLogNamer(std::string_view base_name,
                                     EventLoop *event_loop)
    : LogNamer(event_loop), base_name_(base_name), old_base_name_() {}

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

  const std::string_view separator = base_name_.back() == '/' ? "" : "_";
  const std::string filename = absl::StrCat(
      base_name_, separator, config_sha256, ".bfbs", extension_, temp_suffix_);

  std::unique_ptr<DetachedBufferWriter> writer =
      std::make_unique<DetachedBufferWriter>(filename, encoder_factory_());

  writer->QueueSizedFlatbuffer(header->Release());

  if (!writer->ran_out_of_space()) {
    all_filenames_.emplace_back(filename);
  }
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
      OpenDataWriter();
    }
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

  NewDataWriter data_writer(this, source_node,
                            [this, channel](NewDataWriter *data_writer) {
                              OpenWriter(channel, data_writer);
                            },
                            [this](NewDataWriter *data_writer) {
                              CloseWriter(&data_writer->writer);
                            });
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

  NewDataWriter data_writer(this, node,
                            [this, channel](NewDataWriter *data_writer) {
                              OpenForwardedTimestampWriter(channel,
                                                           data_writer);
                            },
                            [this](NewDataWriter *data_writer) {
                              CloseWriter(&data_writer->writer);
                            });
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
    OpenDataWriter();
  }
  return data_writer_.get();
}

void MultiNodeLogNamer::Close() {
  data_writers_.clear();
  data_writer_.reset();
}

void MultiNodeLogNamer::ResetStatistics() {
  for (std::pair<const Channel *const, NewDataWriter> &data_writer :
       data_writers_) {
    if (!data_writer.second.writer) continue;
    data_writer.second.writer->ResetStatistics();
  }
  if (data_writer_) {
    data_writer_->writer->ResetStatistics();
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
  CreateBufferWriter(filename, &data_writer->writer);
}

void MultiNodeLogNamer::OpenWriter(const Channel *channel,
                                   NewDataWriter *data_writer) {
  const std::string filename = absl::StrCat(
      CHECK_NOTNULL(channel->source_node())->string_view(), "_data",
      channel->name()->string_view(), "/", channel->type()->string_view(),
      ".part", data_writer->parts_index(), ".bfbs", extension_);
  CreateBufferWriter(filename, &data_writer->writer);
}

void MultiNodeLogNamer::OpenDataWriter() {
  if (!data_writer_) {
    data_writer_ = std::make_unique<NewDataWriter>(
        this, node_,
        [this](NewDataWriter *writer) {
          std::string name;
          if (node() != nullptr) {
            name = absl::StrCat(name, node()->name()->string_view(), "_");
          }
          absl::StrAppend(&name, "data.part", writer->parts_index(), ".bfbs",
                          extension_);
          CreateBufferWriter(name, &writer->writer);
        },
        [this](NewDataWriter *data_writer) {
          CloseWriter(&data_writer->writer);
        });
  }
}

void MultiNodeLogNamer::CreateBufferWriter(
    std::string_view path, std::unique_ptr<DetachedBufferWriter> *destination) {
  if (ran_out_of_space_) {
    // Refuse to open any new files, which might skip data. Any existing files
    // are in the same folder, which means they're on the same filesystem, which
    // means they're probably going to run out of space and get stuck too.
    if (!destination->get()) {
      // But avoid leaving a nullptr writer if we're out of space when
      // attempting to open the first file.
      *destination = std::make_unique<DetachedBufferWriter>(
          DetachedBufferWriter::already_out_of_space_t());
    }
    return;
  }
  const std::string_view separator = base_name_.back() == '/' ? "" : "_";
  const std::string filename =
      absl::StrCat(base_name_, separator, path, temp_suffix_);
  if (!destination->get()) {
    if (ran_out_of_space_) {
      *destination = std::make_unique<DetachedBufferWriter>(
          DetachedBufferWriter::already_out_of_space_t());
      return;
    }
    *destination =
        std::make_unique<DetachedBufferWriter>(filename, encoder_factory_());
    if (!destination->get()->ran_out_of_space()) {
      all_filenames_.emplace_back(path);
    }
    return;
  }

  CloseWriter(destination);
  if (ran_out_of_space_) {
    *destination->get() =
        DetachedBufferWriter(DetachedBufferWriter::already_out_of_space_t());
    return;
  }

  *destination->get() = DetachedBufferWriter(filename, encoder_factory_());
  if (!destination->get()->ran_out_of_space()) {
    all_filenames_.emplace_back(path);
  }
}

void MultiNodeLogNamer::RenameTempFile(DetachedBufferWriter *destination) {
  if (temp_suffix_.empty()) {
    return;
  }
  std::string current_filename = std::string(destination->filename());
  CHECK(current_filename.size() > temp_suffix_.size());
  std::string final_filename =
      current_filename.substr(0, current_filename.size() - temp_suffix_.size());
  int result = rename(current_filename.c_str(), final_filename.c_str());

  // When changing the base name, we rename the log folder while there active
  // buffer writers. Therefore, the name of that active buffer may still refer
  // to the old file location rather than the new one. This minimized changes to
  // existing code.
  if (result != 0 && errno != ENOSPC && !old_base_name_.empty()) {
    auto offset = current_filename.find(old_base_name_);
    if (offset != std::string::npos) {
      current_filename.replace(offset, old_base_name_.length(), base_name_);
    }
    offset = final_filename.find(old_base_name_);
    if (offset != std::string::npos) {
      final_filename.replace(offset, old_base_name_.length(), base_name_);
    }
    result = rename(current_filename.c_str(), final_filename.c_str());
  }

  if (result != 0) {
    if (errno == ENOSPC) {
      ran_out_of_space_ = true;
      return;
    } else {
      PLOG(FATAL) << "Renaming " << current_filename << " to " << final_filename
                  << " failed";
    }
  } else {
    VLOG(1) << "Renamed " << current_filename << " -> " << final_filename;
  }
}

void MultiNodeLogNamer::CloseWriter(
    std::unique_ptr<DetachedBufferWriter> *writer_pointer) {
  DetachedBufferWriter *const writer = writer_pointer->get();
  if (!writer) {
    return;
  }
  const bool was_open = writer->is_open();
  writer->Close();

  if (writer->max_write_time() > max_write_time_) {
    max_write_time_ = writer->max_write_time();
    max_write_time_bytes_ = writer->max_write_time_bytes();
    max_write_time_messages_ = writer->max_write_time_messages();
  }
  total_write_time_ += writer->total_write_time();
  total_write_count_ += writer->total_write_count();
  total_write_messages_ += writer->total_write_messages();
  total_write_bytes_ += writer->total_write_bytes();

  if (writer->ran_out_of_space()) {
    ran_out_of_space_ = true;
    writer->acknowledge_out_of_space();
  }
  if (was_open) {
    RenameTempFile(writer);
  } else {
    CHECK(access(std::string(writer->filename()).c_str(), F_OK) == -1)
        << ": File should not exist: " << writer->filename();
  }
}

}  // namespace logger
}  // namespace aos
