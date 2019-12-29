#ifndef AOS_UNIQUE_MALLOC_PTR_H_
#define AOS_UNIQUE_MALLOC_PTR_H_

#include <memory>

namespace aos {

namespace internal {

// Written as a functor so that it doesn't have to get passed to
// std::unique_ptr's constructor as an argument.
template<typename T, void(*function)(T *)>
class const_wrap {
 public:
  void operator()(const T *ptr) {
    function(const_cast<T *>(ptr));
  }
};

// Wrapper function to deal with the differences between C and C++ (C++ doesn't
// automatically convert T* to void* like C).
template<typename T>
void free_type(T *ptr) { ::free(reinterpret_cast<void *>(ptr)); }

}  // namespace internal

// A std::unique_ptr that should get freed with a C-style free function
// (free(2) by default).
template <typename T, void (*function)(T *) = internal::free_type<T>>
class unique_c_ptr
    : public std::unique_ptr<T, internal::const_wrap<T, function>> {
 public:
  unique_c_ptr(T *value)
      : std::unique_ptr<T, internal::const_wrap<T, function>>(value) {}

  // perfect forwarding of these 2 to make unique_ptr work
  template <typename... Args>
  unique_c_ptr(Args &&... args)
      : std::unique_ptr<T, internal::const_wrap<T, function>>(
            std::forward<Args>(args)...) {}
  template<typename... Args>
  unique_c_ptr<T, function> &operator=(Args&&... args) {
    std::unique_ptr<T, internal::const_wrap<T, function>>::operator=(
        std::forward<Args>(args)...);
    return *this;
  }
};

}  // namespace aos

#endif  // AOS_UNIQUE_MALLOC_PTR_H_
