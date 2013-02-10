#include <memory>

namespace aos {

namespace {

template<typename T, void(*function)(T *)>
void const_wrap(const T *ptr) {
  function(const_cast<T *>(ptr));
}

// Wrapper function to deal with the differences between C and C++ (C++ doesn't
// automatically convert T* to void* like C).
template<typename T>
void free_type(T *ptr) { ::free(reinterpret_cast<void *>(ptr)); }

}  // namespace

// A std::unique_ptr that should get freed with a C-style free function
// (free(2) by default).
template<typename T, void(*function)(T *) = free_type>
class unique_c_ptr : public std::unique_ptr<T, void(*)(const T *)> {
 public:
  unique_c_ptr(T *pointer)
      : std::unique_ptr<T, void(*)(const T *)>(
          pointer, const_wrap<T, function>) {}
};

}  // namespace aos
