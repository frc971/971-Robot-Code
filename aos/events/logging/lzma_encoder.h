#ifndef AOS_EVENTS_LOGGING_LZMA_ENCODER_H_
#define AOS_EVENTS_LOGGING_LZMA_ENCODER_H_

#include "absl/types/span.h"
#include "flatbuffers/flatbuffers.h"
#include "lzma.h"

#include "aos/containers/resizeable_buffer.h"
#include "aos/events/logging/buffer_encoder.h"
#include "aos/events/logging/logger_generated.h"

namespace aos::logger {

// Encodes buffers using liblzma.
class LzmaEncoder final : public DetachedBufferEncoder {
 public:
  // Initializes the LZMA stream and encoder.
  explicit LzmaEncoder(uint32_t compression_preset);
  LzmaEncoder(const LzmaEncoder &) = delete;
  LzmaEncoder(LzmaEncoder &&other) = delete;
  LzmaEncoder &operator=(const LzmaEncoder &) = delete;
  LzmaEncoder &operator=(LzmaEncoder &&other) = delete;
  // Gracefully shuts down the encoder.
  ~LzmaEncoder() final;

  void Encode(flatbuffers::DetachedBuffer &&in) final;
  void Finish() final;
  void Clear(int n) final;
  std::vector<absl::Span<const uint8_t>> queue() const final;
  size_t queued_bytes() const final;
  size_t total_bytes() const final { return total_bytes_; }
  size_t queue_size() const final { return queue_.size(); }

 private:
  static constexpr size_t kEncodedBufferSizeBytes{1024};

  void RunLzmaCode(lzma_action action);

  lzma_stream stream_;
  uint32_t compression_preset_;
  std::vector<ResizeableBuffer> queue_;
  bool finished_ = false;
  // Total bytes that resulted from encoding raw data since the last call to
  // Reset.
  size_t total_bytes_ = 0;
};

// Decompresses data with liblzma.
class LzmaDecoder final : public DataDecoder {
 public:
  explicit LzmaDecoder(std::string_view filename);
  LzmaDecoder(const LzmaDecoder &) = delete;
  LzmaDecoder(LzmaDecoder &&other) = delete;
  LzmaDecoder &operator=(const LzmaDecoder &) = delete;
  LzmaDecoder &operator=(LzmaDecoder &&other) = delete;
  ~LzmaDecoder();

  size_t Read(uint8_t *begin, uint8_t *end) final;

 private:
  // Size of temporary buffer to use.
  static constexpr size_t kBufSize{256 * 1024};

  // Temporary buffer for storing compressed data.
  ResizeableBuffer compressed_data_;
  // Used for reading data from the file.
  DummyDecoder dummy_decoder_;
  // Stream for decompression.
  lzma_stream stream_;
  // The current action. This is LZMA_RUN until we've run out of data to read
  // from the file.
  lzma_action action_ = LZMA_RUN;
  // Flag that represents whether or not all the data from the file has been
  // successfully decoded.
  bool finished_ = false;
};

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_LZMA_ENCODER_H_
