#include "aos/events/logging/buffer_encoder.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "glog/logging.h"

#include "aos/flatbuffers.h"

namespace aos::logger {

DummyEncoder::DummyEncoder(size_t /*max_message_size*/, size_t buffer_size) {
  // Round up to the nearest page size.
  input_buffer_.reserve(buffer_size);
  return_queue_.resize(1);
}

size_t DummyEncoder::space() const {
  return input_buffer_.capacity() - input_buffer_.size();
}

bool DummyEncoder::HasSpace(size_t request) const { return request <= space(); }

size_t DummyEncoder::Encode(Copier *copy, size_t start_byte) {
  const size_t input_buffer_initial_size = input_buffer_.size();

  size_t expected_write_size =
      std::min(input_buffer_.capacity() - input_buffer_initial_size,
               copy->size() - start_byte);
  input_buffer_.resize(input_buffer_initial_size + expected_write_size);
  const size_t written_size =
      copy->Copy(input_buffer_.data() + input_buffer_initial_size, start_byte,
                 expected_write_size + start_byte);

  total_bytes_ += written_size;

  return written_size;
}

void DummyEncoder::Clear(const int n) {
  CHECK_GE(n, 0);
  CHECK_LE(static_cast<size_t>(n), queue_size());
  if (n != 0) {
    input_buffer_.resize(0u);
  }
}

absl::Span<const absl::Span<const uint8_t>> DummyEncoder::queue() {
  if (input_buffer_.size() != 0) {
    return_queue_[0] =
        absl::Span<const uint8_t>(input_buffer_.data(), input_buffer_.size());
    return return_queue_;
  } else {
    return absl::Span<const absl::Span<const uint8_t>>();
  }
}

size_t DummyEncoder::queued_bytes() const { return input_buffer_.size(); }

DummyDecoder::DummyDecoder(std::string_view filename)
    : filename_(filename), fd_(open(filename_.c_str(), O_RDONLY | O_CLOEXEC)) {
  PCHECK(fd_ != -1) << ": Failed to open " << filename;
}

DummyDecoder::~DummyDecoder() {
  int status = close(fd_);
  if (status != 0) {
    PLOG(ERROR) << "DummyDecoder: Failed to close file";
  }
}

size_t DummyDecoder::Read(uint8_t *begin, uint8_t *end) {
  if (end_of_file_) {
    return 0;
  }
  const ssize_t count = read(fd_, begin, end - begin);
  PCHECK(count >= 0) << ": Failed to read from file";
  if (count == 0) {
    end_of_file_ = true;
  }
  return static_cast<size_t>(count);
}

}  // namespace aos::logger
