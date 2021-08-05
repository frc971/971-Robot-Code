#include "aos/events/logging/buffer_encoder.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "glog/logging.h"

namespace aos::logger {

void DummyEncoder::Encode(flatbuffers::DetachedBuffer &&in) {
  CHECK(in.data()) << ": Encode called with nullptr.";

  total_bytes_ += in.size();
  queue_.emplace_back(std::move(in));
}

void DummyEncoder::Clear(const int n) {
  CHECK_GE(n, 0);
  CHECK_LE(static_cast<size_t>(n), queue_size());
  queue_.erase(queue_.begin(), queue_.begin() + n);
}

std::vector<absl::Span<const uint8_t>> DummyEncoder::queue() const {
  std::vector<absl::Span<const uint8_t>> queue;
  queue.reserve(queue_.size());
  for (const auto &buffer : queue_) {
    queue.emplace_back(buffer.data(), buffer.size());
  }
  return queue;
}

size_t DummyEncoder::queued_bytes() const {
  size_t bytes = 0;
  for (const auto &buffer : queue_) {
    bytes += buffer.size();
  }
  return bytes;
}

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
