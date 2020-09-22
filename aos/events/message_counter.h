#ifndef AOS_EVENTS_MESSAGE_COUNTER_H_
#define AOS_EVENTS_MESSAGE_COUNTER_H_

#include "aos/events/event_loop.h"

namespace aos {
namespace testing {

// Simple class to count messages on a channel easily.  This only counts
// messages published while running.
template <typename T>
class MessageCounter {
 public:
  MessageCounter(aos::EventLoop *event_loop, std::string_view name) {
    event_loop->MakeNoArgWatcher<T>(name, [this]() { ++count_; });
  }

  // Returns the number of messages seen so far.
  size_t count() const { return count_; }

 private:
  size_t count_ = 0;
};

}  // namespace testing
}  // namespace aos

#endif  // AOS_EVENTS_MESSAGE_COUNTER_H_
