#ifndef AOS_CONTAINERS_ERROR_LIST_H_
#define AOS_CONTAINERS_ERROR_LIST_H_

#include <iostream>

#include "flatbuffers/flatbuffers.h"

#include "aos/containers/sized_array.h"

namespace aos {

// A de-duplicated sorted array based on SizedArray
// For keeping a list of errors that a subsystem has thrown
// to publish them in a Status message.
// It is designed to use flatbuffer enums, and use the reserved fields MAX and
// MIN to automatically determine how much capacity it needs to have.
template <typename T>
class ErrorList {
 private:
  using array = SizedArray<T, static_cast<size_t>(T::MAX) -
                                  static_cast<size_t>(T::MIN) + 1>;
  array array_;

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

  constexpr ErrorList() = default;
  ErrorList(const ErrorList &) = default;
  ErrorList(ErrorList &&) = default;
  ErrorList(const flatbuffers::Vector<T> &array) : array_() {
    for (auto it = array.begin(); it < array.end(); it++) {
      array_.push_back(*it);
    }
    std::sort(array_.begin(), array_.end());
  };

  ErrorList &operator=(const ErrorList &) = default;
  ErrorList &operator=(ErrorList &&) = default;

  bool operator==(const ErrorList &other) const {
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
  bool operator!=(const ErrorList &other) const { return !(*this == other); }

  reference at(size_t i) { return array_.at(i); }
  const_reference at(size_t i) const { return array_.at(i); }

  reference operator[](size_t i) { return array_[i]; }
  const_reference operator[](size_t i) const { return array_[i]; }

  reference front() { return array_.front(); }
  const_reference front() const { return array_.front(); }

  reference back() { return array_.back(); }
  const_reference back() const { return array_.back(); }

  T *data() { return array_.data(); }
  const T *data() const { return array_.data(); }

  iterator begin() { return array_.begin(); }
  const_iterator begin() const { return array_.begin(); }
  const_iterator cbegin() const { return array_.cbegin(); }

  iterator end() { return array_.end(); }
  const_iterator end() const { return array_.end(); }
  const_iterator cend() const { return array_.cend(); }

  reverse_iterator rbegin() { return array_.rbegin(); }
  const_reverse_iterator rbegin() const { return array_.rbegin(); }
  const_reverse_iterator crbegin() const { return array_.crbegin(); }

  reverse_iterator rend() { return array_.rend(); }
  const_reverse_iterator rend() const { return array_.rend(); }
  const_reverse_iterator crend() const { return array_.crend(); }

  bool empty() const { return array_.empty(); }
  bool full() const { return array_.full(); }

  size_t size() const { return array_.size(); }
  constexpr size_t max_size() const { return array_.max_size(); }

  void Clear(const T t) {
    iterator index = std::find(array_.begin(), array_.end(), t);
    if (index != array_.end()) {
      array_.erase(index);
    }
  }

  void Set(const T t) {
    iterator position = std::lower_bound(array_.begin(), array_.end(), t);

    // if it found something, and that something is the same, just leave it
    if (position != array_.end() && *position == t) {
      return;
    }

    // key doesn't already exist
    array_.insert(position, t);
  }

  bool Has(const T t) {
    return std::binary_search(array_.begin(), array_.end(), t);
  }

  flatbuffers::Offset<flatbuffers::Vector<T>> ToFlatbuffer(
      flatbuffers::FlatBufferBuilder *fbb) const {
    return fbb->CreateVector(array_.data(), array_.size());
  }
};  // namespace aos

}  // namespace aos

#endif  // AOS_CONTAINERS_ERROR_LIST_H_
