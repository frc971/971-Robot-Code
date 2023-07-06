#ifndef AOS_EVENTS_LOGGING_LZMA_ENCODER_H_
#define AOS_EVENTS_LOGGING_LZMA_ENCODER_H_

#include <condition_variable>
#include <mutex>
#include <string_view>
#include <thread>

#include "absl/types/span.h"
#include "flatbuffers/flatbuffers.h"

#include "aos/containers/resizeable_buffer.h"
#include "aos/events/logging/buffer_encoder.h"
#include "aos/events/logging/logger_generated.h"
#include "lzma.h"

namespace aos::logger {

// Encodes buffers using liblzma.
class LzmaEncoder final : public DataEncoder {
 public:
  static constexpr std::string_view kExtension = ".xz";

  // Initializes the LZMA stream and encoder.  The block size is the block size
  // used by the multithreaded encoder for batching.  A block size of 0 tells
  // lzma to pick it's favorite block size.
  explicit LzmaEncoder(size_t max_message_size, uint32_t compression_preset,
                       size_t block_size = 0);
  LzmaEncoder(const LzmaEncoder &) = delete;
  LzmaEncoder(LzmaEncoder &&other) = delete;
  LzmaEncoder &operator=(const LzmaEncoder &) = delete;
  LzmaEncoder &operator=(LzmaEncoder &&other) = delete;
  // Gracefully shuts down the encoder.
  ~LzmaEncoder() final;

  bool HasSpace(size_t /*request*/) const override {
    // Since the underlying lzma encoder handles buffering, we always have
    // space.
    return true;
  }
  size_t space() const final { return input_buffer_.capacity(); }
  size_t Encode(Copier *copy, size_t start_byte) final;
  void Finish() final;
  void Clear(int n) final;
  absl::Span<const absl::Span<const uint8_t>> queue() final;
  size_t queued_bytes() const final;
  size_t total_bytes() const final { return total_bytes_; }
  size_t queue_size() const final { return queue_.size(); }

 private:
  static constexpr size_t kEncodedBufferSizeBytes{1024 * 128};

  void RunLzmaCode(lzma_action action);

  lzma_stream stream_;
  uint32_t compression_preset_;
  std::vector<ResizeableBuffer> queue_;
  // Since we pretty much just allocate a couple of buffers, then allocate and
  // release them over and over with very similar memory usage and without much
  // variation in the peak usage, put the allocate chunks in a free queue to
  // reduce fragmentation.
  std::vector<ResizeableBuffer> free_queue_;
  bool finished_ = false;
  // Total bytes that resulted from encoding raw data since the last call to
  // Reset.
  size_t total_bytes_ = 0;

  // Buffer that messages get coppied into for encoding.
  ResizeableBuffer input_buffer_;

  std::vector<absl::Span<const uint8_t>> return_queue_;
};

// Decompresses data with liblzma.
class LzmaDecoder final : public DataDecoder {
 public:
  static constexpr std::string_view kExtension = ".xz";

  explicit LzmaDecoder(std::unique_ptr<DataDecoder> underlying_decoder,
                       bool quiet = false);
  explicit LzmaDecoder(std::string_view filename, bool quiet = false)
      : LzmaDecoder(std::make_unique<DummyDecoder>(filename), quiet) {}
  LzmaDecoder(const LzmaDecoder &) = delete;
  LzmaDecoder(LzmaDecoder &&other) = delete;
  LzmaDecoder &operator=(const LzmaDecoder &) = delete;
  LzmaDecoder &operator=(LzmaDecoder &&other) = delete;
  ~LzmaDecoder();

  size_t Read(uint8_t *begin, uint8_t *end) final;
  std::string_view filename() const final {
    return underlying_decoder_->filename();
  }

 private:
  // Size of temporary buffer to use.
  static constexpr size_t kBufSize{256 * 1024};

  // Temporary buffer for storing compressed data.
  ResizeableBuffer compressed_data_;
  // Used for reading data from the file.
  std::unique_ptr<DataDecoder> underlying_decoder_;
  // Stream for decompression.
  lzma_stream stream_;
  // The current action. This is LZMA_RUN until we've run out of data to read
  // from the file.
  lzma_action action_ = LZMA_RUN;
  // Flag that represents whether or not all the data from the file has been
  // successfully decoded.
  bool finished_ = false;
  // Flag to signal how quiet to be when logging potential issues around
  // truncation.
  const bool quiet_ = false;
};

// Decompresses data with liblzma in a new thread, up to a maximum queue
// size. Calls to Read() will return data from the queue if available,
// or block until more data is queued or the stream finishes.
class ThreadedLzmaDecoder : public DataDecoder {
 public:
  explicit ThreadedLzmaDecoder(std::string_view filename, bool quiet = false)
      : ThreadedLzmaDecoder(std::make_unique<DummyDecoder>(filename), quiet) {}
  explicit ThreadedLzmaDecoder(std::unique_ptr<DataDecoder> underlying_decoder,
                               bool quiet = false);
  ThreadedLzmaDecoder(const ThreadedLzmaDecoder &) = delete;
  ThreadedLzmaDecoder &operator=(const ThreadedLzmaDecoder &) = delete;

  ~ThreadedLzmaDecoder();

  size_t Read(uint8_t *begin, uint8_t *end) final;

  std::string_view filename() const final { return decoder_.filename(); }

 private:
  static constexpr size_t kBufSize{256 * 1024};
  static constexpr size_t kQueueSize{8};

  LzmaDecoder decoder_;

  // Queue of decompressed data to return on calls to Read
  std::vector<ResizeableBuffer> decoded_queue_;

  // Mutex to control access to decoded_queue_.
  std::mutex decode_mutex_;
  std::condition_variable continue_decoding_;
  std::condition_variable queue_filled_;

  bool finished_ = false;

  std::thread decode_thread_;
};

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_LZMA_ENCODER_H_
