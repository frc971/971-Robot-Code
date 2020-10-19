#include "aos/events/logging/log_namer.h"

#include <functional>
#include <map>
#include <memory>
#include <string_view>
#include <vector>

#include "absl/strings/str_cat.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/logging/uuid.h"
#include "flatbuffers/flatbuffers.h"
#include "glog/logging.h"

namespace aos {
namespace logger {

void LogNamer::UpdateHeader(
    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
    const UUID &uuid, int parts_index) const {
  header->mutable_message()->mutate_parts_index(parts_index);
  CHECK_EQ(uuid.string_view().size(),
           header->mutable_message()->mutable_parts_uuid()->size());
  std::copy(uuid.string_view().begin(), uuid.string_view().end(),
            reinterpret_cast<char *>(
                header->mutable_message()->mutable_parts_uuid()->Data()));
}

void LocalLogNamer::WriteHeader(
    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
    const Node *node) {
  CHECK_EQ(node, this->node());
  UpdateHeader(header, uuid_, part_number_);
  data_writer_->QueueSpan(header->full_span());
}

DetachedBufferWriter *LocalLogNamer::MakeWriter(const Channel *channel) {
  CHECK(configuration::ChannelIsSendableOnNode(channel, node()))
      << ": " << configuration::CleanedChannelToString(channel);
  return data_writer_.get();
}

void LocalLogNamer::Rotate(
    const Node *node,
    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header) {
  CHECK(node == this->node());
  ++part_number_;
  *data_writer_ = std::move(*OpenDataWriter());
  UpdateHeader(header, uuid_, part_number_);
  data_writer_->QueueSpan(header->full_span());
}

DetachedBufferWriter *LocalLogNamer::MakeTimestampWriter(
    const Channel *channel) {
  CHECK(configuration::ChannelIsReadableOnNode(channel, node_))
      << ": Message is not delivered to this node.";
  CHECK(node_ != nullptr) << ": Can't log timestamps in a single node world";
  CHECK(configuration::ConnectionDeliveryTimeIsLoggedOnNode(channel, node_,
                                                            node_))
      << ": Delivery times aren't logged for this channel on this node.";
  return data_writer_.get();
}

DetachedBufferWriter *LocalLogNamer::MakeForwardedTimestampWriter(
    const Channel * /*channel*/, const Node * /*node*/) {
  LOG(FATAL) << "Can't log forwarded timestamps in a singe log file.";
  return nullptr;
}

MultiNodeLogNamer::MultiNodeLogNamer(std::string_view base_name,
                                     const Configuration *configuration,
                                     const Node *node)
    : LogNamer(node), base_name_(base_name), configuration_(configuration) {}

MultiNodeLogNamer::~MultiNodeLogNamer() {
  if (!ran_out_of_space_) {
    // This handles renaming temporary files etc.
    Close();
  }
}

void MultiNodeLogNamer::WriteHeader(
    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
    const Node *node) {
  if (node == this->node()) {
    if (!data_writer_.writer) {
      OpenDataWriter();
    }
    UpdateHeader(header, data_writer_.uuid, data_writer_.part_number);
    data_writer_.writer->QueueSpan(header->full_span());
  } else {
    for (std::pair<const Channel *const, DataWriter> &data_writer :
         data_writers_) {
      if (node == data_writer.second.node) {
        UpdateHeader(header, data_writer.second.uuid,
                     data_writer.second.part_number);
        data_writer.second.writer->QueueSpan(header->full_span());
      }
    }
  }
}

void MultiNodeLogNamer::Rotate(
    const Node *node,
    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header) {
  if (node == this->node()) {
    if (data_writer_.writer) {
      ++data_writer_.part_number;
    }
    OpenDataWriter();
    UpdateHeader(header, data_writer_.uuid, data_writer_.part_number);
    data_writer_.writer->QueueSpan(header->full_span());
  } else {
    for (std::pair<const Channel *const, DataWriter> &data_writer :
         data_writers_) {
      if (node == data_writer.second.node) {
        ++data_writer.second.part_number;
        data_writer.second.rotate(data_writer.first, &data_writer.second);
        UpdateHeader(header, data_writer.second.uuid,
                     data_writer.second.part_number);
        data_writer.second.writer->QueueSpan(header->full_span());
      }
    }
  }
}

DetachedBufferWriter *MultiNodeLogNamer::MakeWriter(const Channel *channel) {
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
    if (!data_writer_.writer) {
      OpenDataWriter();
    }
    return data_writer_.writer.get();
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

  DataWriter data_writer;
  data_writer.node = source_node;
  data_writer.rotate = [this](const Channel *channel, DataWriter *data_writer) {
    OpenWriter(channel, data_writer);
  };
  data_writer.rotate(channel, &data_writer);

  return data_writers_.insert(std::make_pair(channel, std::move(data_writer)))
      .first->second.writer.get();
}

DetachedBufferWriter *MultiNodeLogNamer::MakeForwardedTimestampWriter(
    const Channel *channel, const Node *node) {
  // See if we can read the data on this node at all.
  const bool is_readable =
      configuration::ChannelIsReadableOnNode(channel, this->node());
  CHECK(is_readable) << ": " << configuration::CleanedChannelToString(channel);

  CHECK(data_writers_.find(channel) == data_writers_.end());

  if (std::find(nodes_.begin(), nodes_.end(), node) == nodes_.end()) {
    nodes_.emplace_back(node);
  }

  DataWriter data_writer;
  data_writer.node = node;
  data_writer.rotate = [this](const Channel *channel, DataWriter *data_writer) {
    OpenForwardedTimestampWriter(channel, data_writer);
  };
  data_writer.rotate(channel, &data_writer);

  return data_writers_.insert(std::make_pair(channel, std::move(data_writer)))
      .first->second.writer.get();
}

DetachedBufferWriter *MultiNodeLogNamer::MakeTimestampWriter(
    const Channel *channel) {
  bool log_delivery_times = false;
  if (this->node() != nullptr) {
    log_delivery_times = configuration::ConnectionDeliveryTimeIsLoggedOnNode(
        channel, this->node(), this->node());
  }
  if (!log_delivery_times) {
    return nullptr;
  }

  if (!data_writer_.writer) {
    OpenDataWriter();
  }
  return data_writer_.writer.get();
}

void MultiNodeLogNamer::Close() {
  for (std::pair<const Channel *const, DataWriter> &data_writer :
       data_writers_) {
    CloseWriter(&data_writer.second.writer);
    data_writer.second.writer.reset();
  }
  CloseWriter(&data_writer_.writer);
  data_writer_.writer.reset();
}

void MultiNodeLogNamer::ResetStatistics() {
  for (std::pair<const Channel *const, DataWriter> &data_writer :
       data_writers_) {
    data_writer.second.writer->ResetStatistics();
  }
  if (data_writer_.writer) {
    data_writer_.writer->ResetStatistics();
  }
  max_write_time_ = std::chrono::nanoseconds::zero();
  max_write_time_bytes_ = -1;
  max_write_time_messages_ = -1;
  total_write_time_ = std::chrono::nanoseconds::zero();
  total_write_count_ = 0;
  total_write_messages_ = 0;
  total_write_bytes_ = 0;
}

void MultiNodeLogNamer::OpenForwardedTimestampWriter(const Channel *channel,
                                                     DataWriter *data_writer) {
  std::string filename =
      absl::StrCat("timestamps", channel->name()->string_view(), "/",
                   channel->type()->string_view(), ".part",
                   data_writer->part_number, ".bfbs", extension_);
  CreateBufferWriter(filename, &data_writer->writer);
}

void MultiNodeLogNamer::OpenWriter(const Channel *channel,
                                   DataWriter *data_writer) {
  const std::string filename = absl::StrCat(
      CHECK_NOTNULL(channel->source_node())->string_view(), "_data",
      channel->name()->string_view(), "/", channel->type()->string_view(),
      ".part", data_writer->part_number, ".bfbs", extension_);
  CreateBufferWriter(filename, &data_writer->writer);
}

void MultiNodeLogNamer::OpenDataWriter() {
  std::string name;
  if (node() != nullptr) {
    name = absl::StrCat(name, node()->name()->string_view(), "_");
  }
  absl::StrAppend(&name, "data.part", data_writer_.part_number, ".bfbs",
                  extension_);
  CreateBufferWriter(name, &data_writer_.writer);
}

void MultiNodeLogNamer::CreateBufferWriter(
    std::string_view path, std::unique_ptr<DetachedBufferWriter> *destination) {
  if (ran_out_of_space_) {
    // Refuse to open any new files, which might skip data. Any existing files
    // are in the same folder, which means they're on the same filesystem, which
    // means they're probably going to run out of space and get stuck too.
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
  const std::string current_filename = std::string(destination->filename());
  CHECK(current_filename.size() > temp_suffix_.size());
  const std::string final_filename =
      current_filename.substr(0, current_filename.size() - temp_suffix_.size());
  const int result = rename(current_filename.c_str(), final_filename.c_str());
  if (result != 0) {
    if (errno == ENOSPC) {
      ran_out_of_space_ = true;
      return;
    } else {
      PLOG(FATAL) << "Renaming " << current_filename << " to " << final_filename
                  << " failed";
    }
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
