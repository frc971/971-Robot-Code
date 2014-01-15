#ifndef __AOS_MESSAGES_QUEUE_HOLDER_H_
#define __AOS_MESSAGES_QUEUE_HOLDER_H_

#include <stddef.h>
#include <string.h>

#include <algorithm>

#include "aos/common/control_loop/Timing.h"
#include "aos/common/byteorder.h"
#include "aos/common/time.h"
#include "aos/common/type_traits.h"
#include "aos/common/logging/logging.h"
#ifndef __VXWORKS__
#include "aos/linux_code/ipc_lib/queue.h"
#endif

namespace aos {

// Specializations of TypeOperator and QueueBuilder that are actually
// implemented are created in the generated code for all message value types.
// These specializations have the same functions declared in the actual types.

// Defines a way for types to be manipulated.
template<typename T> class TypeOperator {
 public:
  // Sets all fields to their default constructor.
  static void Zero(T &t_);
  // Returns the size of buffer NToH and HToN use.
  static size_t Size();
  // Converts everything from network to host byte order.
  // input must have Size() bytes available in it.
  static void NToH(char *input, T &t_);
  // Converts everything from host to network byte order and puts it into output.
  // output must have Size() bytes available in it.
  static void HToN(const T &t_, char *output);
  // Creates a string with the names and values of all the fields.
  // The return value might will be to a static buffer.
  static const char *Print(const T &t_);
};

template<typename T> class QueueHolder;

// An easy way to set values for queue messages.
// Each specialization has chainable setter functions for building a message
// of type T to put into a queue (like QueueBuilder<T> &field(int value);).
template<class T> class QueueBuilder {
 public:
  QueueBuilder(QueueHolder<T> &holder);
  bool Send();
};

// Internal class to make implementing identical behavior with structs that go
// into queues easier. Also a central location for the documentation.
//
// When compiled for the cRIO, everything except Clear does nothing (and Get
// turns into just a Clear) which means that the internal T instance is the only one.
// Also, the internal instance becomes static.
//
// To look at the current message, you Get a copy and then View the result. 
// To make changes, you modify the message that you can View (after possibly
// Clearing it) and then you Send it. You can also just Clear the message, put
// data into it, and then Send it. A way to modify the local message is using
// the Builder function.
// Note that there is no way to avoid potentially overwriting other changes between
// when you Get one and when you Send it (mainly applicable to 1-length queues).
//
// T must be POD and have a "timespec set_time;" field.
//
// This first class doesn't have the builder; QueueHolder does.
#define aos_check_rv __attribute__((warn_unused_result))
template<typename T> class QueueHolderNoBuilder {
#ifndef __VXWORKS__
  Queue *const queue_;
  static_assert(shm_ok<T>::value, "T must be able to"
                " go through shared memory and memcpy");
  T t_;
#else
  static T t_;
#endif
 public:
#ifndef __VXWORKS__
  explicit QueueHolderNoBuilder(Queue *queue) : queue_(queue) {}
#else
  QueueHolderNoBuilder() {}
#endif
  // Gets the current value and stores it in View().
  // check_time is whether or not to check to see if the last time a value was Sent
  // was too long ago (returns false if it was)
  // Returns true if View() is now the current value.
  // IMPORTANT: If this function returns false, the contents of View() are
  // either the same as they were before or (if check_time is true) the current
  // message. That is why it creates compile-time warnings if the return value
  // is not checked.
  bool Get(bool check_time) aos_check_rv;
  // Looks at the current value. Starts out undefined.
  // If the last Get call returned false, then this the contents of the
  // return value are undefined.
#ifdef __VXWORKS__
  static
#endif
  inline T &View() { return t_; }
  // Clears (calls the default constructor of) all the fields of the current
  // Goal.
  void Clear() { TypeOperator<T>::Zero(t_); }
  // Sends the current value. Does not affect the current value.
  // Returns whether or not the Send succeeded.
  bool Send();
  // Returns a string containing the values of all the fields in the current
  // value.
  // The return value is valid until Print is called again. The class owns the
  // memory.
  const char *Print() const { return TypeOperator<T>::Print(t_); }
};

template<typename T>
bool QueueHolderNoBuilder<T>::Get(bool check_time) {
#ifdef __VXWORKS__
  (void)check_time;
  return true;
#else
  const T *msg = static_cast<const T *>(aos_queue_read_msg(queue_,
                                                           PEEK | NON_BLOCK));
  if (msg == NULL) {
    return false;
  }
  static_assert(sizeof(t_) == sizeof(*msg), "something is wrong here");
  memcpy(&t_, msg, sizeof(t_));
  aos_queue_free_msg(queue_, msg);
  if (check_time && !((time::Time::Now() - time::Time(t_.set_time)) > time::Time::InMS(45))) {
    LOG(WARNING, "too long since last Set msg={%s}\n", Print());
    return false;
  } else {
    return true;
  }
#endif
}
template<typename T>
bool QueueHolderNoBuilder<T>::Send() {
#ifndef __VXWORKS__
  T *msg = static_cast<T *>(aos_queue_get_msg(queue_));
  if (msg == NULL) {
    return false;
  }
  static_assert(sizeof(*msg) == sizeof(t_), "something is wrong here");
  memcpy(msg, &t_, sizeof(t_));
  msg->set_time = ::aos::time::Time::Now().ToTimespec();

  return aos_queue_write_msg_free(queue_, msg, OVERRIDE) == 0;
#else
  return true;
#endif
}
#ifdef __VXWORKS__
template<typename T> T QueueHolderNoBuilder<T>::t_;
#endif
template<typename T> class QueueHolder : public QueueHolderNoBuilder<T> {
  QueueBuilder<T> builder_;
 public:
#ifndef __VXWORKS__
  explicit QueueHolder(Queue *queue) : QueueHolderNoBuilder<T>(queue),
    builder_(*this) {}
#else
  QueueHolder() : builder_(*this) {}
#endif
  // Clears the current Goal and returns an object that allows setting various
  // fields with chained method calls and then calling Send() on it.
  QueueBuilder<T> &Builder() {
    QueueHolderNoBuilder<T>::Clear();
    return builder_;
  }
};

}  // namespace aos

#endif
