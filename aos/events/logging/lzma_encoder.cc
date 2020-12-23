#include "aos/events/logging/lzma_encoder.h"

#include "glog/logging.h"

namespace aos::logger {
namespace {

// Returns true if `status` is not an error code, false if it is recoverable, or
// otherwise logs the appropriate error message and crashes.
bool LzmaCodeIsOk(lzma_ret status, std::string_view filename = "") {
  switch (status) {
    case LZMA_OK:
    case LZMA_STREAM_END:
      return true;
    case LZMA_MEM_ERROR:
      LOG(FATAL) << "Memory allocation failed:" << status;
    case LZMA_OPTIONS_ERROR:
      LOG(FATAL) << "The given compression preset or decompression options are "
                    "not supported: "
                 << status;
    case LZMA_UNSUPPORTED_CHECK:
      LOG(FATAL) << "The given check type is not supported: " << status;
    case LZMA_PROG_ERROR:
      LOG(FATAL) << "One or more of the parameters have values that will never "
                    "be valid: "
                 << status;
    case LZMA_MEMLIMIT_ERROR:
      LOG(FATAL) << "Decoder needs more memory than allowed by the specified "
                    "memory usage limit: "
                 << status;
    case LZMA_FORMAT_ERROR:
      if (filename.empty()) {
        LOG(FATAL) << "File format not recognized: " << status;
      } else {
        LOG(FATAL) << "File format of " << filename
                   << " not recognized: " << status;
      }
    case LZMA_DATA_ERROR:
      LOG(WARNING) << "Compressed file is corrupt: " << status;
      return false;
    case LZMA_BUF_ERROR:
      LOG(WARNING) << "Compressed file is truncated or corrupt: " << status;
      return false;
    default:
      LOG(FATAL) << "Unexpected return value: " << status;
  }
}

}  // namespace

LzmaEncoder::LzmaEncoder(const uint32_t compression_preset)
    : stream_(LZMA_STREAM_INIT), compression_preset_(compression_preset) {
  CHECK_GE(compression_preset_, 0u)
      << ": Compression preset must be in the range [0, 9].";
  CHECK_LE(compression_preset_, 9u)
      << ": Compression preset must be in the range [0, 9].";

  lzma_ret status =
      lzma_easy_encoder(&stream_, compression_preset_, LZMA_CHECK_CRC64);
  CHECK(LzmaCodeIsOk(status));
  stream_.avail_out = 0;
  VLOG(2) << "LzmaEncoder: Initialization succeeded.";
}

LzmaEncoder::~LzmaEncoder() { lzma_end(&stream_); }

void LzmaEncoder::Encode(flatbuffers::DetachedBuffer &&in) {
  CHECK(in.data()) << ": Encode called with nullptr.";

  stream_.next_in = in.data();
  stream_.avail_in = in.size();

  RunLzmaCode(LZMA_RUN);
}

void LzmaEncoder::Finish() { RunLzmaCode(LZMA_FINISH); }

void LzmaEncoder::Clear(const int n) {
  CHECK_GE(n, 0);
  CHECK_LE(static_cast<size_t>(n), queue_size());
  queue_.erase(queue_.begin(), queue_.begin() + n);
  if (queue_.empty()) {
    stream_.next_out = nullptr;
    stream_.avail_out = 0;
  }
}

std::vector<absl::Span<const uint8_t>> LzmaEncoder::queue() const {
  std::vector<absl::Span<const uint8_t>> queue;
  if (queue_.empty()) {
    return queue;
  }
  for (size_t i = 0; i < queue_.size() - 1; ++i) {
    queue.emplace_back(
        absl::MakeConstSpan(queue_.at(i).data(), queue_.at(i).size()));
  }
  // For the last buffer in the queue, we must account for the possibility that
  // the buffer isn't full yet.
  queue.emplace_back(absl::MakeConstSpan(
      queue_.back().data(), queue_.back().size() - stream_.avail_out));
  return queue;
}

size_t LzmaEncoder::queued_bytes() const {
  size_t bytes = queue_size() * kEncodedBufferSizeBytes;
  // Subtract the bytes that the encoder hasn't filled yet.
  bytes -= stream_.avail_out;
  return bytes;
}

void LzmaEncoder::RunLzmaCode(lzma_action action) {
  CHECK(!finished_);

  // This is to keep track of how many bytes resulted from encoding this input
  // buffer.
  size_t last_avail_out = stream_.avail_out;

  while (stream_.avail_in > 0 || action == LZMA_FINISH) {
    // If output buffer is full, create a new one, queue it up, and resume
    // encoding. This could happen in the first call to Encode after
    // construction or a Reset, or when an input buffer is large enough to fill
    // more than one output buffer.
    if (stream_.avail_out == 0) {
      queue_.emplace_back();
      queue_.back().resize(kEncodedBufferSizeBytes);
      stream_.next_out = queue_.back().data();
      stream_.avail_out = kEncodedBufferSizeBytes;
      // Update the byte count.
      total_bytes_ += last_avail_out;
      last_avail_out = stream_.avail_out;
    }

    // Encode the data.
    lzma_ret status = lzma_code(&stream_, action);
    CHECK(LzmaCodeIsOk(status));
    if (action == LZMA_FINISH) {
      if (status == LZMA_STREAM_END) {
        // This is returned when lzma_code is all done.
        finished_ = true;
        break;
      }
    } else {
      CHECK(status != LZMA_STREAM_END);
    }
    VLOG(2) << "LzmaEncoder: Encoded chunk.";
  }

  // Update the number of resulting encoded bytes.
  total_bytes_ += last_avail_out - stream_.avail_out;
}

LzmaDecoder::LzmaDecoder(std::string_view filename)
    : dummy_decoder_(filename), stream_(LZMA_STREAM_INIT), filename_(filename) {
  compressed_data_.resize(kBufSize);

  lzma_ret status =
      lzma_stream_decoder(&stream_, UINT64_MAX, LZMA_CONCATENATED);
  CHECK(LzmaCodeIsOk(status)) << "Failed initializing LZMA stream decoder.";
  stream_.avail_out = 0;
  VLOG(2) << "LzmaDecoder: Initialization succeeded.";
}

LzmaDecoder::~LzmaDecoder() { lzma_end(&stream_); }

size_t LzmaDecoder::Read(uint8_t *begin, uint8_t *end) {
  if (finished_) {
    return 0;
  }

  // Write into the given range.
  stream_.next_out = begin;
  stream_.avail_out = end - begin;
  // Keep decompressing until we run out of buffer space.
  while (stream_.avail_out > 0) {
    if (action_ == LZMA_RUN && stream_.avail_in == 0) {
      // Read more bytes from the file if we're all out.
      const size_t count =
          dummy_decoder_.Read(compressed_data_.begin(), compressed_data_.end());
      if (count == 0) {
        // No more data to read in the file, begin the finishing operation.
        action_ = LZMA_FINISH;
      } else {
        stream_.next_in = compressed_data_.data();
        stream_.avail_in = count;
      }
    }
    // Decompress the data.
    const lzma_ret status = lzma_code(&stream_, action_);
    // Return if we're done.
    if (status == LZMA_STREAM_END) {
      CHECK_EQ(action_, LZMA_FINISH)
          << ": Got LZMA_STREAM_END when action wasn't LZMA_FINISH";
      finished_ = true;
      return (end - begin) - stream_.avail_out;
    }

    // If we fail to decompress, give up.  Return everything that has been
    // produced so far.
    if (!LzmaCodeIsOk(status, filename_)) {
      finished_ = true;
      LOG(WARNING) << filename_ << " is truncated or corrupted.";
      return (end - begin) - stream_.avail_out;
    }
  }
  return end - begin;
}

}  // namespace aos::logger
