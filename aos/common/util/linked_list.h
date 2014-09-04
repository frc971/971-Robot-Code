#ifndef AOS_COMMON_UTIL_LINKED_LIST_H_
#define AOS_COMMON_UTIL_LINKED_LIST_H_

#include <functional>

#include "aos/common/transaction.h"

namespace aos {
namespace util {

// Handles manipulating an intrusive linked list. T must look like the
// following:
// struct T {
//   ...
//   T *next;
//   ...
// };
// This class doesn't deal with creating or destroying them, so
// constructors/destructors/other members variables/member functions are all
// fine, but the next pointer must be there for this class to work.
// This class will handle all manipulations of next. It does not need to be
// initialized before calling Add and should not be changed afterwards.
// next can (and probably should) be private if the appropriate instantiation of
// this class is friended.
template <class T>
class LinkedList {
 public:
  T *head() const { return head_; }

  bool Empty() const { return head() == nullptr; }

  void Add(T *t) {
    Add<0>(t, nullptr);
  }

  // restore_points (if non-null) will be used so the operation can be safely
  // reverted at any point.
  template <int number_works>
  void Add(T *t, transaction::WorkStack<transaction::RestorePointerWork,
                                        number_works> *restore_pointers) {
    if (restore_pointers != nullptr) restore_pointers->AddWork(&t->next);
    t->next = head();
    if (restore_pointers != nullptr) restore_pointers->AddWork(&head_);
    head_ = t;
  }

  void Remove(T *t) {
    Remove<0>(t, nullptr);
  }

  // restore_points (if non-null) will be used so the operation can be safely
  // reverted at any point.
  template <int number_works>
  void Remove(T *t, transaction::WorkStack<transaction::RestorePointerWork,
                                           number_works> *restore_pointers) {
    T **pointer = &head_;
    while (*pointer != nullptr) {
      if (*pointer == t) {
        if (restore_pointers != nullptr) {
          restore_pointers->AddWork(pointer);
        }
        *pointer = t->next;
        return;
      }
      pointer = &(*pointer)->next;
    }
    LOG(FATAL, "%p is not in the list\n", t);
  }

  // Calls function for each element of the list.
  // function can modify these elements in any way except touching the next
  // pointer (including by calling other methods of this object).
  void Each(::std::function<void(T *)> function) const {
    T *c = head();
    while (c != nullptr) {
      T *const next = c->next;
      function(c);
      c = next;
    }
  }

  // Returns the first element of the list where function returns true or
  // nullptr if it returns false for all.
  T *Find(::std::function<bool(const T *)> function) const {
    T *c = head();
    while (c != nullptr) {
      if (function(c)) return c;
      c = c->next;
    }
    return nullptr;
  }

 private:
  T *head_ = nullptr;
};

// Keeps track of something along with a next pointer. Useful for things that
// either have types without next pointers or for storing pointers to things
// that belong in multiple lists.
template <class V>
struct LinkedListReference {
  V item;

 private:
  friend class LinkedList<LinkedListReference>;

  LinkedListReference *next;
};

}  // namespace util
}  // namespace aos

#endif  // AOS_COMMON_UTIL_LINKED_LIST_H_
