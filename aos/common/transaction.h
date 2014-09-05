#ifndef AOS_COMMON_TRANSACTION_H_
#define AOS_COMMON_TRANSACTION_H_

#include <stdint.h>

#include <array>

#include "aos/common/util/compiler_memory_barrier.h"
#include "aos/common/logging/logging.h"

namespace aos {
namespace transaction {

// Manages a LIFO stack of Work objects. Designed to help implement transactions
// by providing a safe way to undo things etc.
//
// number_works Work objects are created statically and then Create is called on
// each as it is added to the stack. When the work should do whatever it does,
// DoWork() will be called. The work objects get no notification when they are
// dropped off of the stack.
//
// Work::DoWork() must be idempotent because it may get called multiple times if
// CompleteWork() is interrupted part of the way through.
//
// This class handles compiler memory barriers etc to make sure only fully
// created works are ever invoked, and each work will be fully created by the
// time AddWork returns. This does not mean it's safe for multiple threads to
// interact with an instance of this class at the same time.
template <class Work, int number_works>
class WorkStack {
 public:
  // Calls DoWork() on all the works that have been added and then removes them
  // all from the stack.
  void CompleteWork() {
    int current = stack_index_;
    while (current > 0) {
      stack_.at(--current).DoWork();
    }
    aos_compiler_memory_barrier();
    stack_index_ = 0;
    aos_compiler_memory_barrier();
  }

  // Drops all works that have been added.
  void DropWork() {
    stack_index_ = 0;
    aos_compiler_memory_barrier();
  }

  // Returns true if we have any works to complete right now.
  bool HasWork() const { return stack_index_ != 0; }

  // Forwards all of its arguments to Work::Create, which it calls on the next
  // work to be added.
  template <class... A>
  void AddWork(A &&... a) {
    if (stack_index_ >= number_works) {
      LOG(FATAL, "too many works\n");
    }
    stack_.at(stack_index_).Create(::std::forward<A>(a)...);
    aos_compiler_memory_barrier();
    ++stack_index_;
    aos_compiler_memory_barrier();
  }

 private:
  // The next index into stack_ for a new work to be added.
  int stack_index_ = 0;
  ::std::array<Work, number_works> stack_;
};

// When invoked, sets *pointer to the value it had when this work was Created.
template <class T>
class RestoreValueWork {
 public:
  void Create(T *pointer) {
    pointer_ = pointer;
    value_ = *pointer;
  }
  void DoWork() {
    *pointer_ = value_;
  }

 private:
  T *pointer_;
  T value_;
};

// Handles the casting necessary to restore any kind of pointer.
class RestorePointerWork : public RestoreValueWork<void *> {
 public:
  template <class T>
  void Create(T **pointer) {
    static_assert(sizeof(T *) == sizeof(void *),
                  "that's a weird pointer");
    RestoreValueWork<void *>::Create(reinterpret_cast<void **>(pointer));
  }
};

}  // namespace transaction
}  // namespace aos

#endif  // AOS_COMMON_TRANSACTION_H_
