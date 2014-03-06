#include <memory>

#include "aos/atom_code/ipc_lib/queue.h"

namespace aos {
namespace internal {

template<typename T>
class queue_free {
 public:
  queue_free(RawQueue *queue) : queue_(queue) {}

  void operator()(const T *message) {
    queue_->FreeMessage(static_cast<const void *>(message));
  }

 private:
  RawQueue *const queue_;
};

}  // namespace internal

template<typename T>
class unique_message_ptr : public ::std::unique_ptr<T, ::aos::internal::queue_free<T>> {
 public:
  unique_message_ptr(RawQueue *queue, T *message = NULL)
      : ::std::unique_ptr<T, ::aos::internal::queue_free<T>>(message, ::aos::internal::queue_free<T>(queue)) {}

  // Perfectly forward this so that the move functionality of ::std::unique_ptr
  // works.
  template <typename... Args>
  unique_message_ptr<T> &operator=(Args &&... args) {
        ::std::unique_ptr<T, ::aos::internal::queue_free<T>>::operator=(
            ::std::forward<Args>(args)...);
        return *this;
  }
};

}  // namespace aos
