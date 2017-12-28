#ifndef _AOS_EVENTS_EVENT_LOOP_TMPL_H_
#define _AOS_EVENTS_EVENT_LOOP_TMPL_H_

#include <type_traits>
#include "aos/events/event-loop.h"

namespace aos {

// From a watch functor, this will extract the message type of the argument.
// This is the template forward declaration, and it extracts the call operator
// as a PTMF to be used by the following specialization.
template <class T>
struct watch_message_type_trait
    : watch_message_type_trait<decltype(&T::operator())> {};

// From a watch functor, this will extract the message type of the argument.
// This is the template specialization.
template <class ClassType, class ReturnType, class A1>
struct watch_message_type_trait<ReturnType (ClassType::*)(A1) const> {
  using message_type = typename std::decay<A1>::type;
};

template <typename T>
typename Sender<T>::Message Sender<T>::MakeMessage() {
  return Message(sender_.get());
}

template <typename Watch>
void EventLoop::MakeWatcher(const std::string &path, Watch &&w) {
  using T = typename watch_message_type_trait<Watch>::message_type;

  return MakeRawWatcher(path, QueueTypeInfo::Get<T>(),
                        [w](const Message *message) {
                          w(*reinterpret_cast<const T *>(message));
                        });
}

}  // namespace aos

#endif  // _AOS_EVENTS_EVENT_LOOP_TMPL_H
