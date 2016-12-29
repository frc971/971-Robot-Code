#ifndef _AOS_VISION_EVENTS_INTRUSIVE_FREE_LIST_H_
#define _AOS_VISION_EVENTS_INTRUSIVE_FREE_LIST_H_

namespace aos {
namespace events {

// Hey! Maybe you want a doubly linked list that frees things for you!
// This allows the entry to delete itself, removing it from the list, or
// when the list gets destructed all the entries get destructed.
//
// class MyType : public intrusive_free_list<MyType>::element {
//  public:
//   MyType(int i, intrusive_free_list<MyType>* list)
//       : element(list, this), i_(i) {}
//   ~MyType() { printf("o%d\n", i_); }
//  private:
//   int i_;
// };
//
// void test_fn() {
//   intrusive_free_list<MyType> free_list;
//   auto o0 = new MyType(0, &free_list);
//   auto o1 = new MyType(1, &free_list);
//   auto o2 = new MyType(2, &free_list);
//   auto o3 = new MyType(2, &free_list);
//
//   delete o2;
//   delete o1;
// }
//
// // This will print:
// o2
// o1
// o3
// o0
//
// Note that anything that was not manually freed (o0, o3) is
// freed by the linked list destructor at the end. This ensures everything
// is always destructed even entities not manually destructed.
template <class T>
class intrusive_free_list {
 public:
  class element {
   public:
    element(intrusive_free_list<T> *list, T *t) : list_(list), prev_(nullptr) {
      next_ = list_->begin_;
      if (next_) next_->prev_ = t;
      list_->begin_ = t;
    }
    ~element() {
      if (next_) next_->prev_ = prev_;
      if (prev_) {
        prev_->next_ = next_;
      } else {
        list_->begin_ = next_;
      }
    }
    T *next() { return next_; }

   private:
    friend class intrusive_free_list<T>;
    // Consider using static casts and a header element to save this pointer.
    intrusive_free_list<T> *list_;
    T *next_;
    T *prev_;
  };
  intrusive_free_list() : begin_(nullptr) {}
  ~intrusive_free_list() {
    while (begin_) delete begin_;
  }
  T *begin() { return begin_; }

 private:
  friend class element;
  T *begin_;
};

}  // namespace events
}  // namespace aos

#endif  // _AOS_VISION_EVENTS_INTRUSIVE_FREE_LIST_H_
