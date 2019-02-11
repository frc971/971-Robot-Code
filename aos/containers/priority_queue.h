#ifndef AOS_CONTAINERS_PRIORITY_QUEUE_H_
#define AOS_CONTAINERS_PRIORITY_QUEUE_H_

#include <array>
#include <iterator>

namespace aos {

// Creates a priority queue which will never exceed a particular size.
// Data: The type of the data to store.
// buffer_size: The max number of Data to store.
// Compare: A comparison function. If std::less were used here, then the
//   smallest element would be discarded first, see
//   https://en.cppreference.com/w/cpp/named_req/Compare
// The lowest priority elements will be discarded first to maintain buffer_size.
// Note that as a "queue" this is a bit incomplete because there is no mechanism
// to pop from the queue.
template <typename Data, size_t buffer_size, typename Compare>
class PriorityQueue {
 public:
  class iterator {
   public:
    explicit iterator(PriorityQueue *queue, size_t idx)
        : queue_(queue), idx_(idx) {}
    iterator &operator++() {
      idx_ = queue_->next_idx(idx_);
      return *this;
    }
    iterator operator++(int) {
      iterator retval = *this;
      ++(*this);
      return retval;
    }
    iterator &operator--() {
      idx_ = queue_->prev_idx(idx_);
      return *this;
    }
    iterator operator--(int) {
      iterator retval = *this;
      --(*this);
      return retval;
    }
    bool operator==(iterator other) const {
      return queue_ == other.queue_ && idx_ == other.idx_;
    }
    bool operator!=(iterator other) const { return !(*this == other); }
    Data &operator*() { return queue_->get(idx_); }
    Data *operator->() { return &queue_->get(idx_); }

   private:
    PriorityQueue *queue_;
    size_t idx_;
  };

  constexpr PriorityQueue() {}

  // Inserts data into the queue and returns an iterator for the inserted
  // element. If the queue was already full, then the lowest priority element
  // will be discarded. If data is lower priority than all the current elements
  // and the queue is full, then data is ignored and end() is returned.
  // PushFromBottom starts the search from the bottom of the queue.
  // TODO(james): If performance becomes an issue, improve search.
  iterator PushFromBottom(const Data &data) {
    size_t lower_idx = buffer_size;
    size_t upper_idx = bottom_;
    // Find our spot in the queue:
    while (upper_idx != buffer_size && !cmp_(data, list_[upper_idx].data)) {
      lower_idx = upper_idx;
      upper_idx = list_[upper_idx].upper_idx;
      if (upper_idx == buffer_size) {
        break;
      }
    }
    return InsertInList(data, lower_idx, upper_idx);
  }

  size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }
  bool full() const { return size_ == buffer_size; }

  // Removes all the elements from the queue:
  void clear() {
    size_ = 0;
    bottom_ = buffer_size;
    top_ = buffer_size;
  }

  Data &top() { return list_[top_].data; }
  const Data &top() const { return list_[top_].data; }
  Data &get(size_t idx) { return list_[idx].data; }
  const Data &get(size_t idx) const { return list_[idx].data; }
  iterator begin() { return iterator(this, bottom_); }
  iterator end() { return iterator(this, buffer_size); }

  // Gets the index of the next (higher valued) element in the queue.
  size_t next_idx(size_t idx) const { return list_[idx].upper_idx; }
  // Gets the index of the previous (lower valued) element in the queue.
  size_t prev_idx(size_t idx) const { return list_[idx].lower_idx; }
 private:
  struct Datum {
    // A list element with data and the indices of the next highest/lowest
    // priority elements.
    Data data;
    // Values of buffer_size indicate that we are at the beginning or end of
    // the queue.
    size_t lower_idx = buffer_size;
    size_t upper_idx = buffer_size;
  };

  // Insert an element above lower_idx and below upper_idx.
  iterator InsertInList(const Data &data, size_t lower_idx, size_t upper_idx) {
    // For inserting new elements, when we are initially filling the queue we
    // will increment upwards in the array; once full, we just evict the
    // lowest priority element.
    size_t insertion_idx = size();
    if (full()) {
      if (upper_idx == bottom_) {
        // this item is lower priority than everyone else, don't insert it.
        return end();
      }
      // Eject the lowest priority element.
      insertion_idx = bottom_;
      if (lower_idx == insertion_idx) {
        lower_idx = buffer_size;
      }
      --size_;
      bottom_ = list_[bottom_].upper_idx;
      list_[bottom_].lower_idx = buffer_size;
    }
    if (upper_idx != buffer_size) {
      list_[upper_idx].lower_idx = insertion_idx;
    }
    if (lower_idx != buffer_size) {
      list_[lower_idx].upper_idx = insertion_idx;
    }
    if (bottom_ == upper_idx) {
      bottom_ = insertion_idx;
    }
    if (top_ == lower_idx) {
      top_ = insertion_idx;
    }
    list_[insertion_idx].data = data;
    list_[insertion_idx].upper_idx = upper_idx;
    list_[insertion_idx].lower_idx = lower_idx;
    ++size_;
    return iterator(this, insertion_idx);
  }
  ::std::array<Datum, buffer_size> list_;
  // Index of the bottom and top of the queue.
  size_t bottom_ = buffer_size, top_ = buffer_size;
  // Number of elements currently in the queue.
  size_t size_ = 0;
  Compare cmp_;
};
}  // namespace aos

#endif  // AOS_CONTAINERS_PRIORITY_QUEUE_H_
