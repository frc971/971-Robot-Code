#ifndef AOS_COMMON_SCOPED_PTR_H_
#define AOS_COMMON_SCOPED_PTR_H_

#include "aos/common/macros.h"

namespace aos {

// A simple scoped_ptr implementation that works under both linux and vxworks.
template<typename T>
class scoped_ptr {
 public:
  typedef T element_type;
  
  explicit scoped_ptr(T *p = NULL) : p_(p) {}
  ~scoped_ptr() {
    delete p_;
  }

  T &operator*() const { return *p_; }
  T *operator->() const { return p_; }
  T *get() const { return p_; }

  operator bool() const { return p_ != NULL; }

  void swap(scoped_ptr<T> &other) {
    T *temp = other.p_;
    other.p_ = p_;
    p_ = other.p_;
  }
  void reset(T *p = NULL) {
    if (p_ != NULL) delete p_;
    p_ = p;
  }

 private:
  T *p_;

  DISALLOW_COPY_AND_ASSIGN(scoped_ptr<T>);
};

}  // namespace aos

#endif  // AOS_COMMON_SCOPED_PTR_H_
