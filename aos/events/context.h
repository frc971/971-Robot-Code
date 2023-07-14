#ifndef AOS_EVENTS_CONTEXT_H_
#define AOS_EVENTS_CONTEXT_H_

#include "aos/flatbuffers.h"
#include "aos/time/time.h"
#include "aos/uuid.h"

namespace aos {

// Struct available on Watchers, Fetchers, Timers, and PhasedLoops with context
// about the current message.
struct Context {
  // Time that the message was sent on this node, or the timer was triggered.
  monotonic_clock::time_point monotonic_event_time;
  // Realtime the message was sent on this node.  This is set to min_time for
  // Timers and PhasedLoops.
  realtime_clock::time_point realtime_event_time;

  // The rest are only valid for Watchers and Fetchers.

  // For a single-node configuration, these two are identical to *_event_time.
  // In a multinode configuration, these are the times that the message was
  // sent on the original node.
  monotonic_clock::time_point monotonic_remote_time;
  realtime_clock::time_point realtime_remote_time;

  // Index in the queue.
  uint32_t queue_index;
  // Index into the remote queue.  Useful to determine if data was lost.  In a
  // single-node configuration, this will match queue_index.
  uint32_t remote_queue_index;

  // Size of the data sent.
  size_t size;
  // Pointer to the data.
  const void *data;

  // Index of the message buffer. This will be in [0, NumberBuffers) on
  // read_method=PIN channels, and -1 for other channels.
  //
  // This only tells you about the underlying storage for this message, not
  // anything about its position in the queue. This is only useful for advanced
  // zero-copy use cases, on read_method=PIN channels.
  //
  // This will uniquely identify a message on this channel at a point in time.
  // For senders, this point in time is while the sender has the message. With
  // read_method==PIN, this point in time includes while the caller has access
  // to this context. For other read_methods, this point in time may be before
  // the caller has access to this context, which makes this pretty useless.
  int buffer_index;

  // UUID of the remote node which sent this message, or this node in the case
  // of events which are local to this node.
  UUID source_boot_uuid = UUID::Zero();

  // Efficiently copies the flatbuffer into a FlatbufferVector, allocating
  // memory in the process.  It is vital that T matches the type of the
  // underlying flatbuffer.
  template <typename T>
  FlatbufferVector<T> CopyFlatBuffer() const {
    ResizeableBuffer buffer;
    buffer.resize(size);
    memcpy(buffer.data(), data, size);
    return FlatbufferVector<T>(std::move(buffer));
  }
};

}  // namespace aos

#endif  // AOS_EVENTS_CONTEXT_H_
