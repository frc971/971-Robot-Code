#ifndef AOS_COMMON_MESSAGE_H_
#define AOS_COMMON_MESSAGE_H_

#include "aos/common/time.h"

namespace aos {

struct MessageType;

// This class is a base class for all messages sent over queues.
// All of the methods are overloaded in (generated) subclasses to do the same
// thing for the whole thing.
class Message {
 public:
  // The time that the message was sent at.
  monotonic_clock::time_point sent_time;

  Message() : sent_time(monotonic_clock::min_time) {}

  // Zeros out the time.
  void Zero();
  // Returns the size of the common fields.
  static size_t Size() { return sizeof(sent_time); }

  // Deserializes the common fields from the buffer.
  size_t Deserialize(const char *buffer);
  // Serializes the common fields into the buffer.
  size_t Serialize(char *buffer) const;

  // Populates sent_time with the current time.
  void SetTimeToNow() { sent_time = monotonic_clock::now(); }

  // Writes the contents of the message to the provided buffer.
  size_t Print(char *buffer, int length) const;

  // Compares two messages for equality, excluding their sent_time.
  bool EqualsNoTime(const Message & /*other*/) const { return true; }

  static const MessageType *GetType();
};

// Specializations for the Builders will be automatically generated in the .q.h
// header files with all of the handy builder methods.
template <class T>
class MessageBuilder {
 public:
  typedef T Message;
  bool Send();
};

}  // namespace aos

#endif  // AOS_COMMON_MESSAGE_H_
