#ifndef AOS_RING_BUFFER_H_
#define AOS_RING_BUFFER_H_

#include <array>

namespace aos {

// This is a helper to keep track of some amount of recent data. As you push
// data into the ring buffer, it gets stored. If the buffer becomes full, it
// will start overwriting the oldest data.
template <typename Data, size_t buffer_size>
class RingBuffer {
  template <typename ValueType, typename BufferType>
  struct generic_iterator {
    using iterator_category = ::std::random_access_iterator_tag;
    using value_type = ValueType;
    using difference_type = ::std::ptrdiff_t;
    using pointer = value_type *;
    using reference = value_type &;

    explicit generic_iterator(BufferType *buffer, size_t index)
        : buffer_(buffer), index_(index) {}

    generic_iterator &operator++() {
      ++index_;
      return *this;
    }
    generic_iterator operator++(int) {
      const generic_iterator retval = *this;
      ++(*this);
      return retval;
    }
    generic_iterator &operator--() {
      --index_;
      return *this;
    }
    generic_iterator operator--(int) {
      const generic_iterator retval = *this;
      --(*this);
      return retval;
    }

    generic_iterator &operator+=(ptrdiff_t i) {
      index_ += i;
      return *this;
    }
    generic_iterator operator+(ptrdiff_t i) const {
      generic_iterator retval = *this;
      retval += i;
      return retval;
    }
    generic_iterator &operator-=(ptrdiff_t i) {
      index_ -= i;
      return *this;
    }
    generic_iterator operator-(ptrdiff_t i) const {
      generic_iterator retval = *this;
      retval -= i;
      return retval;
    }

    ptrdiff_t operator-(generic_iterator other) const {
      return index_ - other.index_;
    }

    bool operator==(generic_iterator other) const {
      return buffer_ == other.buffer_ && index_ == other.index_;
    }
    bool operator!=(generic_iterator other) const { return !(*this == other); }

    bool operator<(generic_iterator other) const {
      return buffer_ == other.buffer_ && index_ < other.index_;
    }
    bool operator>(generic_iterator other) const {
      return buffer_ == other.buffer_ && index_ > other.index_;
    }
    bool operator<=(generic_iterator other) const {
      return buffer_ == other.buffer_ && index_ <= other.index_;
    }
    bool operator>=(generic_iterator other) const {
      return buffer_ == other.buffer_ && index_ >= other.index_;
    }

    reference operator*() const { return (*buffer_)[index_]; }
    pointer operator->() const { return &(*buffer_)[index_]; }

    reference operator[](difference_type i) const {
      return (*buffer_)[index_ + i];
    }

   private:
    BufferType *buffer_;
    size_t index_;
  };

 public:
  using iterator = generic_iterator<Data, RingBuffer>;
  using const_iterator = generic_iterator<const Data, const RingBuffer>;

  static constexpr size_t kBufferSize = buffer_size;

  constexpr RingBuffer() {}
  ~RingBuffer() { Reset(); }

  // Add an item to the RingBuffer, overwriting the oldest element if necessary.
  //
  // Invalidates the end iterator.
  void Push(Data data) {
    if (full()) {
      (*this)[0] = std::move(data);
      oldest_ = (oldest_ + 1) % buffer_size;
    } else {
      new (&(*this)[size_]) Data(std::move(data));
      ++size_;
    }
  }

  // Removes the oldest element.
  //
  // Invalidates all iterators.
  void Shift() {
    (*this)[0].~Data();
    oldest_ = (oldest_ + 1) % buffer_size;
    --size_;
  }

  // Return the value of the index requested, adjusted so that the RingBuffer
  // contains the oldest element first and the newest last.
  Data &operator[](size_t index) {
#if defined(__cpp_lib_launder) && __cpp_lib_launder >= 201606
    return *std::launder(
        reinterpret_cast<Data *>(&data_[(oldest_ + index) % buffer_size]));
#else
    // TODO(brian): Remove this when all our compilers are 17 or newer.
    return *reinterpret_cast<Data *>(&data_[(oldest_ + index) % buffer_size]);
#endif
  }

  const Data &operator[](size_t index) const {
#if defined(__cpp_lib_launder) && __cpp_lib_launder >= 201606
    return *std::launder(reinterpret_cast<const Data *>(
        &data_[(oldest_ + index) % buffer_size]));
#else
    // TODO(brian): Remove this when all our compilers are 17 or newer.
    return *reinterpret_cast<const Data *>(
        &data_[(oldest_ + index) % buffer_size]);
#endif
  }

  // Returns the capacity of the RingBuffer
  size_t capacity() const { return buffer_size; }

  // Returns the number of elements stored in the RingBuffer
  size_t size() const { return size_; }

  // Is the RingBuffer empty or full?
  bool empty() const { return size_ == 0; }

  bool full() const { return size_ == buffer_size; }

  // Clears all the data out of the buffer.
  //
  // Invalidates all iterators.
  void Reset() {
    while (!empty()) {
      Shift();
    }
  }

  iterator begin() { return iterator(this, 0); }
  iterator end() { return iterator(this, size()); }

  const_iterator begin() const { return const_iterator(this, 0); }
  const_iterator end() const { return const_iterator(this, size()); }

 private:
  using DataStorage =
      typename std::aligned_storage<sizeof(Data), alignof(Data)>::type;
  std::array<DataStorage, buffer_size> data_;

  // Oldest contains the oldest item added to the RingBuffer which will be
  // the next one to be overwritten
  size_t oldest_ = 0;
  size_t size_ = 0;
};

}  // namespace aos

#endif  // AOS_RING_BUFFER_H_
