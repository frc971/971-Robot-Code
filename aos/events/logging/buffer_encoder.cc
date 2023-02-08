#include "aos/events/logging/buffer_encoder.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "aos/flatbuffers.h"
#include "glog/logging.h"

namespace aos::logger {

DummyEncoder::DummyEncoder(size_t max_buffer_size) {
  // TODO(austin): This is going to end up writing > 128k chunks, not 128k
  // chunks exactly.  If we really want to, we could make it always write 128k
  // chunks by only exposing n * 128k chunks as we go.  This might improve write
  // performance, then again, it might have no effect if the kernel is combining
  // writes...
  constexpr size_t kWritePageSize = 128 * 1024;
  // Round up to the nearest page size.
  input_buffer_.reserve(
      ((max_buffer_size + kWritePageSize - 1) / kWritePageSize) *
      kWritePageSize);
  return_queue_.resize(1);
}

bool DummyEncoder::HasSpace(size_t request) const {
  return request + input_buffer_.size() < input_buffer_.capacity();
}

void DummyEncoder::Encode(Copier *copy) {
  DCHECK(HasSpace(copy->size()));
  const size_t input_buffer_initial_size = input_buffer_.size();

  input_buffer_.resize(input_buffer_initial_size + copy->size());
  const size_t written_size = copy->Copy(
      input_buffer_.data() + input_buffer_initial_size, 0, copy->size());
  DCHECK_EQ(written_size, copy->size());

  total_bytes_ += written_size;
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
