#include "aos/events/logging/lzma_encoder.h"

#include "glog/logging.h"

DEFINE_int32(lzma_threads, 1, "Number of threads to use for encoding");

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
      VLOG(1) << "Compressed file is corrupt: " << status;
      return false;
    case LZMA_BUF_ERROR:
      VLOG(1) << "Compressed file is truncated or corrupt: " << status;
      return false;
    default:
      LOG(FATAL) << "Unexpected return value: " << status;
  }
}

}  // namespace

LzmaEncoder::LzmaEncoder(size_t max_message_size,
                         const uint32_t compression_preset, size_t block_size)
    : stream_(LZMA_STREAM_INIT), compression_preset_(compression_preset) {
  CHECK_GE(compression_preset_, 0u)
      << ": Compression preset must be in the range [0, 9].";
  CHECK_LE(compression_preset_, 9u)
      << ": Compression preset must be in the range [0, 9].";

  if (FLAGS_lzma_threads <= 1) {
    lzma_ret status =
        lzma_easy_encoder(&stream_, compression_preset_, LZMA_CHECK_CRC64);
    CHECK(LzmaCodeIsOk(status));
  } else {
    lzma_mt mt_options;
    memset(&mt_options, 0, sizeof(mt_options));
    mt_options.threads = FLAGS_lzma_threads;
    mt_options.block_size = block_size;
    // Compress for at most 100 ms before relinquishing control back to the main
    // thread.
    mt_options.timeout = 100;
    mt_options.preset = compression_preset_;
    mt_options.filters = nullptr;
    mt_options.check = LZMA_CHECK_CRC64;
    lzma_ret status = lzma_stream_encoder_mt(&stream_, &mt_options);
    CHECK(LzmaCodeIsOk(status));
  }

  stream_.avail_out = 0;
  VLOG(2) << "LzmaEncoder: Initialization succeeded.";

  // TODO(austin): We don't write the biggest messages very often.  Is it more
  // efficient to allocate if we go over a threshold to keep the static memory
  // in use smaller, or just allocate the worst case like we are doing here?
  input_buffer_.resize(max_message_size);
}

LzmaEncoder::~LzmaEncoder() { lzma_end(&stream_); }

size_t LzmaEncoder::Encode(Copier *copy, size_t start_byte) {
  const size_t copy_size = copy->size();
  // LZMA compresses the data as it goes along, copying the compressed results
  // into another buffer.  So, there's no need to store more than one message
  // since lzma is going to take it from here.
  CHECK_LE(copy_size, input_buffer_.size());

  CHECK_EQ(copy->Copy(input_buffer_.data(), start_byte, copy_size - start_byte),
           copy_size - start_byte);

  stream_.next_in = input_buffer_.data();
  stream_.avail_in = copy_size;

  RunLzmaCode(LZMA_RUN);

  return copy_size - start_byte;
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

absl::Span<const absl::Span<const uint8_t>> LzmaEncoder::queue() {
  return_queue_.clear();
  if (queue_.empty()) {
    return return_queue_;
  }
  return_queue_.reserve(queue_.size());
  for (size_t i = 0; i < queue_.size() - 1; ++i) {
    return_queue_.emplace_back(
        absl::MakeConstSpan(queue_.at(i).data(), queue_.at(i).size()));
  }
  // For the last buffer in the queue, we must account for the possibility that
  // the buffer isn't full yet.
  return_queue_.emplace_back(absl::MakeConstSpan(
      queue_.back().data(), queue_.back().size() - stream_.avail_out));
  return return_queue_;
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

LzmaDecoder::LzmaDecoder(std::unique_ptr<DataDecoder> underlying_decoder,
                         bool quiet)
    : underlying_decoder_(std::move(underlying_decoder)),
      stream_(LZMA_STREAM_INIT),
      quiet_(quiet) {
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
      const size_t count = underlying_decoder_->Read(compressed_data_.begin(),
                                                     compressed_data_.end());
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
    if (!LzmaCodeIsOk(status, filename())) {
      finished_ = true;
      if (status == LZMA_DATA_ERROR) {
        if (!quiet_ || VLOG_IS_ON(1)) {
          LOG(WARNING) << filename() << " is corrupted.";
        }
      } else if (status == LZMA_BUF_ERROR) {
        if (!quiet_ || VLOG_IS_ON(1)) {
          LOG(WARNING) << filename() << " is truncated or corrupted.";
        }
      } else {
        LOG(FATAL) << "Unknown error " << status << " when reading "
                   << filename();
      }
      return (end - begin) - stream_.avail_out;
    }
  }
  return end - begin;
}

ThreadedLzmaDecoder::ThreadedLzmaDecoder(
    std::unique_ptr<DataDecoder> underlying_decoder, bool quiet)
    : decoder_(std::move(underlying_decoder), quiet), decode_thread_([this] {
        std::unique_lock lock(decode_mutex_);
        while (true) {
          // Wake if the queue is too small or we are finished.
          continue_decoding_.wait(lock, [this] {
            return decoded_queue_.size() < kQueueSize || finished_;
          });

          if (finished_) {
            return;
          }

          while (true) {
            CHECK(!finished_);
            // Release our lock on the queue before doing decompression work.
            lock.unlock();

            ResizeableBuffer buffer;
            buffer.resize(kBufSize);

            const size_t bytes_read =
                decoder_.Read(buffer.begin(), buffer.end());
            buffer.resize(bytes_read);

            // Relock the queue and move the new buffer to the end. This should
            // be fast. We also need to stay locked when we wait().
            lock.lock();
            if (bytes_read > 0) {
              decoded_queue_.emplace_back(std::move(buffer));
            } else {
              finished_ = true;
            }

            // If we've filled the queue or are out of data, go back to sleep.
            if (decoded_queue_.size() >= kQueueSize || finished_) {
              break;
            }
          }

          // Notify main thread in case it was waiting for us to queue more
          // data.
          queue_filled_.notify_one();
        }
      }) {}

ThreadedLzmaDecoder::~ThreadedLzmaDecoder() {
  // Wake up decode thread so it can return.
  {
    std::scoped_lock lock(decode_mutex_);
    finished_ = true;
  }
  continue_decoding_.notify_one();
  decode_thread_.join();
}

size_t ThreadedLzmaDecoder::Read(uint8_t *begin, uint8_t *end) {
  std::unique_lock lock(decode_mutex_);

  // Strip any empty buffers
  for (auto iter = decoded_queue_.begin(); iter != decoded_queue_.end();) {
    if (iter->size() == 0) {
      iter = decoded_queue_.erase(iter);
    } else {
      ++iter;
    }
  }

  // If the queue is empty, sleep until the decoder thread has produced another
  // buffer.
  if (decoded_queue_.empty()) {
    continue_decoding_.notify_one();
    queue_filled_.wait(lock,
                       [this] { return finished_ || !decoded_queue_.empty(); });
    if (finished_ && decoded_queue_.empty()) {
      return 0;
    }
  }
  // Sanity check if the queue is empty and we're not finished.
  CHECK(!decoded_queue_.empty()) << "Decoded queue unexpectedly empty";

  ResizeableBuffer &front_buffer = decoded_queue_.front();

  // Copy some data from our working buffer to the requested destination.
  const std::size_t bytes_requested = end - begin;
  const std::size_t bytes_to_copy =
      std::min(bytes_requested, front_buffer.size());
  memcpy(begin, front_buffer.data(), bytes_to_copy);
  front_buffer.erase_front(bytes_to_copy);

  // Ensure the decoding thread wakes up if the queue isn't full.
  if (!finished_ && decoded_queue_.size() < kQueueSize) {
    continue_decoding_.notify_one();
  }

  return bytes_to_copy;
}

}  // namespace aos::logger
