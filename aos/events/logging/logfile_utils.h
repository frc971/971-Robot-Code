#ifndef AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_
#define AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_

#include <sys/uio.h>

#include <string_view>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/events/logging/logger_generated.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {
namespace logger {

enum class LogType : uint8_t {
  // The message originated on this node and should be logged here.
  kLogMessage,
  // The message originated on another node, but only the delivery times are
  // logged here.
  kLogDeliveryTimeOnly,
  // The message originated on another node. Log it and the delivery times
  // together.  The message_gateway is responsible for logging any messages
  // which didn't get delivered.
  kLogMessageAndDeliveryTime
};


// This class manages efficiently writing a sequence of detached buffers to a
// file.  It queues them up and batches the write operation.
class DetachedBufferWriter {
 public:
  DetachedBufferWriter(std::string_view filename);
  ~DetachedBufferWriter();

  // TODO(austin): Snappy compress the log file if it ends with .snappy!

  // Queues up a finished FlatBufferBuilder to be written.  Steals the detached
  // buffer from it.
  void QueueSizedFlatbuffer(flatbuffers::FlatBufferBuilder *fbb);
  // Queues up a detached buffer directly.
  void QueueSizedFlatbuffer(flatbuffers::DetachedBuffer &&buffer);

  // Triggers data to be provided to the kernel and written.
  void Flush();

 private:
  int fd_ = -1;

  // Size of all the data in the queue.
  size_t queued_size_ = 0;

  // List of buffers to flush.
  std::vector<flatbuffers::DetachedBuffer> queue_;
  // List of iovecs to use with writev.  This is a member variable to avoid
  // churn.
  std::vector<struct iovec> iovec_;
};

// Packes a message pointed to by the context into a MessageHeader.
flatbuffers::Offset<MessageHeader> PackMessage(
    flatbuffers::FlatBufferBuilder *fbb, const Context &context,
    int channel_index, LogType log_type);

// TODO(austin): 3 objects:
// 1) log chunk reader.  Returns span.
// 2) Sorted message header reader.  Returns sorted messages.
// 3) LogReader, which does all the registration.
//
// Then, we can do a multi-node sim which forwards data nicely, try logging it, and then try replaying it.

// Optimization:
//   Allocate the 256k blocks like we do today.  But, refcount them with shared_ptr pointed to by the messageheader that is returned.  This avoids the copy.

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_
