#ifndef AOS_EVENTS_LOGGING_SNAPPY_ENCODER_H_
#define AOS_EVENTS_LOGGING_SNAPPY_ENCODER_H_

#include <string_view>

#include "absl/types/span.h"
#include "aos/containers/resizeable_buffer.h"
#include "aos/events/logging/buffer_encoder.h"
#include "aos/events/logging/logger_generated.h"
#include "flatbuffers/flatbuffers.h"
#include "snappy-sinksource.h"
#include "snappy.h"

namespace aos::logger {

// Encodes buffers using snappy.
class SnappyEncoder final : public DataEncoder {
 public:
  explicit SnappyEncoder(size_t max_message_size, size_t chunk_size = 32768);

  void Encode(Copier *copy) final;

  void Finish() final;
  void Clear(int n) final;
  absl::Span<const absl::Span<const uint8_t>> queue() final;
  size_t queued_bytes() const final;
  bool HasSpace(size_t /*request*/) const final {
    // Since the output always mallocs space, we have infinite output space.
    return true;
  }
  size_t total_bytes() const final { return total_bytes_; }
  size_t queue_size() const final { return queue_.size(); }

 private:
  class DetachedBufferSource : public snappy::Source {
   public:
    DetachedBufferSource(size_t buffer_size);
    size_t Available() const final;
    const char *Peek(size_t *length) final;
    void Skip(size_t n) final;
    void Append(Copier *copy);

    uint32_t accumulated_checksum() const {
      return accumulated_checksum_.value();
    }

    void ResetAccumulatedChecksum() { accumulated_checksum_.reset(); }

   private:
    ResizeableBuffer data_;
    size_t index_into_first_buffer_ = 0;
    std::optional<uint32_t> accumulated_checksum_;
  };

  // Flushes buffer_source_ and stores the compressed buffer in queue_.
  void EncodeCurrentBuffer();

  // To queue up data:
  // 1) When Encode is called, we use AppendBuffer to store the DetachedBuffer
  //    in buffer_source_.
  // 2) Once we've queued up at least chunk_size_ data in buffer_source_, we
  //    use snappy to compress all the data. This flushes everything out of
  //    buffer_source_ and adds a single buffer to queue_. Note that we do
  //    not split up flatbuffer buffers to ensure that we produce chunks of
  //    exactly chunk_size_ uncompressed data--if we get a 1MB DetachedBuffer
  //    we will compress it all at once.
  // 3) queue_ is the data that is actually read by queue() and cleared by
  //    Clear() to be written to disk.
  const size_t chunk_size_;
  DetachedBufferSource buffer_source_;
  std::vector<ResizeableBuffer> queue_;

  std::vector<absl::Span<const uint8_t>> return_queue_;
  size_t total_bytes_ = 0;
};

// Decompresses data with snappy.
class SnappyDecoder final : public DataDecoder {
 public:
  static constexpr std::string_view kExtension = ".sz";

  explicit SnappyDecoder(std::unique_ptr<DataDecoder> underlying_decoder)
      : underlying_decoder_(std::move(underlying_decoder)) {}
  explicit SnappyDecoder(std::string_view filename)
      : SnappyDecoder(std::make_unique<DummyDecoder>(filename)) {}

  size_t Read(uint8_t *begin, uint8_t *end) final;
  std::string_view filename() const final {
    return underlying_decoder_->filename();
  }

 private:
  // decoder to use for reading data out of the file itself.
  std::unique_ptr<DataDecoder> underlying_decoder_;
  // Buffer to use for reading data from the file. This being a member variable
  // is purely an optimization to avoid constant reallocations on every call to
  // Read().
  ResizeableBuffer compressed_buffer_;
  // Buffer of any uncompressed data that we've read but which hasn't yet been
  // consumed by a call to Read().
  ResizeableBuffer uncompressed_buffer_;

  size_t total_output_ = 0;
};

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_SNAPPY_ENCODER_H_
