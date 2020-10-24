#ifndef AOS_EVENTS_LOGGING_BUFFER_ENCODER_H_
#define AOS_EVENTS_LOGGING_BUFFER_ENCODER_H_

#include "absl/types/span.h"
#include "flatbuffers/flatbuffers.h"

#include "aos/events/logging/logger_generated.h"

namespace aos::logger {

// Interface to encode DetachedBuffers as they are written to a file.
//
// The interface to a file is represented as a series of buffers, appropriate to
// pass to writev(2). The backing storage for these is managed internally so
// subclasses can handle resizing/moving as desired. Implementations must
// enqueue as many of these backing buffers as demanded, leaving it up to the
// caller to write them to a file when desired.
class DetachedBufferEncoder {
 public:
  virtual ~DetachedBufferEncoder() = default;

  // Encodes and enqueues the given DetachedBuffer.
  //
  // Note that in may be moved-from, or it may be left unchanged, depending on
  // what makes the most sense for a given implementation.
  virtual void Encode(flatbuffers::DetachedBuffer &&in) = 0;

  // If this returns true, the encoder may be bypassed by writing directly to
  // the file.
  virtual bool may_bypass() const { return false; }

  // Finalizes the encoding process. After this, queue_size() represents the
  // full extent of data which will be written to this file.
  //
  // Encode may not be called after this method.
  virtual void Finish() = 0;

  // Clears the first n encoded buffers from the queue.
  virtual void Clear(int n) = 0;

  // Returns a view of the queue of encoded buffers.
  virtual std::vector<absl::Span<const uint8_t>> queue() const = 0;

  // Returns the total number of of bytes currently queued up.
  virtual size_t queued_bytes() const = 0;

  // Returns the cumulative number of bytes which have been queued. This
  // includes data which has been removed via Clear.
  virtual size_t total_bytes() const = 0;

  // Returns the number of elements in the queue.
  virtual size_t queue_size() const = 0;
};

// This class does not encode the data. It just claims ownership of the raw data
// and queues it up as is.
class DummyEncoder final : public DetachedBufferEncoder {
 public:
  DummyEncoder() {}
  DummyEncoder(const DummyEncoder &) = delete;
  DummyEncoder(DummyEncoder &&other) = delete;
  DummyEncoder &operator=(const DummyEncoder &) = delete;
  DummyEncoder &operator=(DummyEncoder &&other) = delete;
  ~DummyEncoder() override = default;

  // No encoding happens, the raw data is queued up as is.
  void Encode(flatbuffers::DetachedBuffer &&in) final;
  bool may_bypass() const final { return true; }
  void Finish() final {}
  void Clear(int n) final;
  std::vector<absl::Span<const uint8_t>> queue() const final;
  size_t queued_bytes() const final;
  size_t total_bytes() const final { return total_bytes_; }
  size_t queue_size() const final { return queue_.size(); }

 private:
  std::vector<flatbuffers::DetachedBuffer> queue_;
  size_t total_bytes_ = 0;
};

// Interface to decode chunks of data. Implementations of this interface will
// manage opening, reading, and closing the file stream.
class DataDecoder {
 public:
  virtual ~DataDecoder() = default;

  // Reads data into the given range. Returns the number of bytes read.
  //
  // Returns less than end-begin if all bytes have been read. Otherwise, this
  // will always fill the whole range.
  virtual size_t Read(uint8_t *begin, uint8_t *end) = 0;
};

// Simply reads the contents of the file into the target buffer.
class DummyDecoder final : public DataDecoder {
 public:
  explicit DummyDecoder(std::string_view filename);
  DummyDecoder(const DummyDecoder &) = delete;
  DummyDecoder(DummyDecoder &&other) = delete;
  DummyDecoder &operator=(const DummyDecoder &) = delete;
  DummyDecoder &operator=(DummyDecoder &&other) = delete;
  ~DummyDecoder() override;

  size_t Read(uint8_t *begin, uint8_t *end) final;

 private:
  // File descriptor for the log file.
  int fd_;

  // Cached bit for if we have reached the end of the file.  Otherwise we will
  // hammer on the kernel asking for more data each time we send.
  bool end_of_file_ = false;
};

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_BUFFER_ENCODER_H_
