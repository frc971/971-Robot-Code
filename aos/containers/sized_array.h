#ifndef AOS_CONTAINERS_SIZED_ARRAY_H_
#define AOS_CONTAINERS_SIZED_ARRAY_H_

#include <array>

namespace aos {

// An array along with a variable size. This is a simple variable-size container
// with a fixed maximum size.
//
// Note that it default-constructs N T instances at construction time. This
// simplifies the internal bookkeeping a lot (I believe this can be
// all-constexpr in C++17), but makes it a poor choice for complex T.
template <typename T, size_t N>
class SizedArray {
 private:
  using array = std::array<T, N>;

 public:
  using value_type = typename array::value_type;
  using size_type = typename array::size_type;
  using difference_type = typename array::difference_type;
  using reference = typename array::reference;
  using const_reference = typename array::const_reference;
  using pointer = typename array::pointer;
  using const_pointer = typename array::const_pointer;
  using iterator = typename array::iterator;
  using const_iterator = typename array::const_iterator;
  using reverse_iterator = typename array::reverse_iterator;
  using const_reverse_iterator = typename array::const_reverse_iterator;

  constexpr SizedArray() = default;
  SizedArray(const SizedArray &) = default;
  SizedArray(SizedArray &&) = default;
  SizedArray &operator=(const SizedArray &) = default;
  SizedArray &operator=(SizedArray &&) = default;

  bool operator==(const SizedArray &other) const {
    if (other.size() != size()) {
      return false;
    }
    for (size_t i = 0; i < size(); ++i) {
      if (other[i] != (*this)[i]) {
        return false;
      }
    }
    return true;
  }
  bool operator!=(const SizedArray &other) const {
    return !(*this == other);
  }

  reference at(size_t i) {
    check_index(i);
    return array_.at(i);
  }
  const_reference at(size_t i) const {
    check_index(i);
    return array_.at(i);
  }

  reference operator[](size_t i) { return array_[i]; }
  const_reference operator[](size_t i) const { return array_[i]; }

  reference front() { return array_.front(); }
  const_reference front() const { return array_.front(); }

  reference back() { return array_[size_ - 1]; }
  const_reference back() const { return array_[size_ - 1]; }

  T *data() { return array_.data(); }
  const T *data() const { return array_.data(); }

  iterator begin() { return array_.begin(); }
  const_iterator begin() const { return array_.begin(); }
  const_iterator cbegin() const { return array_.cbegin(); }

  iterator end() { return array_.begin() + size_; }
  const_iterator end() const { return array_.begin() + size_; }
  const_iterator cend() const { return array_.cbegin() + size_; }

  reverse_iterator rbegin() { return array_.rend() - size_; }
  const_reverse_iterator rbegin() const { return array_.rend() - size_; }
  const_reverse_iterator crbegin() const { return array_.crend() - size_; }

  reverse_iterator rend() { return array_.rend(); }
  const_reverse_iterator rend() const { return array_.rend(); }
  const_reverse_iterator crend() const { return array_.crend(); }

  bool empty() const { return size_ == 0; }
  bool full() const { return size() == max_size(); }

  size_t size() const { return size_; }
  constexpr size_t max_size() const { return array_.max_size(); }

  void push_back(const T &t) {
    array_.at(size_) = t;
    ++size_;
  }
  void push_back(T &&t) {
    array_.at(size_) = std::move(t);
    ++size_;
  }

  void pop_back() {
    if (empty()) {
      __builtin_trap();
    }
    --size_;
  }

  void clear() { size_ = 0; }

  // These allow access to the underlying storage. The data here may be outside
  // the current logical extents of the container.
  const array &backing_array() const { return array_; }
  array *mutable_backing_array() { return &array_; }
  void set_size(size_t size) { size_ = size; }

 private:
  void check_index(size_t i) const {
    if (i >= size_) {
      __builtin_trap();
    }
  }

  array array_;
  size_t size_ = 0;
};

}  // namespace aos

#endif  // AOS_CONTAINERS_SIZED_ARRAY_H_
