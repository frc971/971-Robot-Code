#ifndef AOS_EVENTS_EVENT_LOOP_TMPL_H_
#define AOS_EVENTS_EVENT_LOOP_TMPL_H_

#include <type_traits>
#include "aos/events/event_loop.h"
#include "glog/logging.h"

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
typename Sender<T>::Builder Sender<T>::MakeBuilder() {
  return Builder(sender_.get(), sender_->data(), sender_->size());
}

template <typename Watch>
void EventLoop::MakeWatcher(const std::string_view channel_name, Watch &&w) {
  using T = typename watch_message_type_trait<Watch>::message_type;
  const Channel *channel = configuration::GetChannel(
      configuration_, channel_name, T::GetFullyQualifiedName(), name());

  CHECK(channel != nullptr)
      << ": Channel { \"name\": \"" << channel_name << "\", \"type\": \""
      << T::GetFullyQualifiedName() << "\" } not found in config.";

  return MakeRawWatcher(
      channel, [this, w](const Context &context, const void *message) {
        context_ = context;
        w(*flatbuffers::GetRoot<T>(reinterpret_cast<const char *>(message)));
      });
}

}  // namespace aos

#endif  // AOS_EVENTS_EVENT_LOOP_TMPL_H
