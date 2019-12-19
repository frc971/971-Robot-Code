#ifndef AOS_IPC_LIB_INDEX_H_
#define AOS_IPC_LIB_INDEX_H_

#include <sys/types.h>
#include <atomic>
#include <string>

namespace aos {
namespace ipc_lib {

struct AtomicQueueIndex;
class AtomicIndex;
class Index;

namespace testing {
class QueueIndexTest;
}  // namespace testing

// There are 2 types of indices in the queue.  1 is the index into the overall
// queue.  If we have sent 1,000,005 messages, this is that number.
//
// The other is the index into the message list.  This is essentially a message
// pointer.  It has some of the lower bits of the queue index encoded into it as
// a safeguard to detect if someone re-used a message out from under us and we
// couldn't tell otherwise.  It is used to map a queue index to a message index.
//
// Each of these index types has an atomic version and a non-atomic.  The atomic
// version is a wrapper around a uint32_t to hang helper functions off of.
// The non-atomic version contains all the logic.  These are all in the header
// file to encourage the compiler to inline aggressively.
//
// The user should very infrequently be manipulating the values of these
// directly.  Use the classes instead to do the heavy lifting.

// Structure for holding the index into the queue.
class QueueIndex {
 public:
  // Returns an invalid queue element which uses a reserved value.
  static QueueIndex Invalid() { return QueueIndex(0xffffffff, 0); }
  // Returns a queue element pointing to 0.
  static QueueIndex Zero(uint32_t count) { return QueueIndex(0, count); }

  // Returns true if the index is valid.
  bool valid() const { return index_ != 0xffffffff; }

  // Returns the modulo base used to wrap to avoid overlapping with the reserved
  // number.
  // max_value is one more than the max value we can store.
  // count is the the number of elements in the queue.
  static constexpr uint32_t MaxIndex(uint32_t max_value, uint32_t count) {
    return (max_value / count) * count;
  }

  // Gets the next index.
  QueueIndex Increment() const {
    return IncrementBy(1u);
  }

  // Gets the nth next element.
  QueueIndex IncrementBy(uint32_t amount) const {
    uint32_t index = index_ + amount;
    uint32_t max_index = MaxIndex(sentinal_value(), count_);

    if (index < index_) {
      // We wrapped.  We are shifting up by 0x100000000 - MaxIndex(...).
      // Which is equivalent to subtracting MaxIndex since everything is modular
      // with a uint32_t.
      index -= max_index;
    }

    // Now, wrap the remainder.
    index = index % max_index;
    return QueueIndex(index, count_);
  }

  // Gets the nth previous element.
  QueueIndex DecrementBy(uint32_t amount) const {
    uint32_t index = index_ - amount;
    if (index > index_) {
      // We wrapped.  We are shifting down by 0x100000000 - MaxIndex(...).
      // Which is equivalent to adding MaxIndex since everything is modular with
      // a uint32_t.
      index += MaxIndex(sentinal_value(), count_);
    }
    return QueueIndex(index, count_);
  }

  // Returns true if the lowest 16 bits of the queue index from the Index could
  // plausibly match this queue index.
  bool IsPlausible(uint16_t queue_index) const {
    return valid() && (queue_index == static_cast<uint16_t>(index_ & 0xffff));
  }

  bool operator==(const QueueIndex other) const {
    return other.index_ == index_;
  }

  bool operator!=(const QueueIndex other) const {
    return other.index_ != index_;
  }

  // Returns the wrapped index into the queue.
  uint32_t Wrapped() const { return index_ % count_; }

  // Returns the raw index.  This should be used very sparingly.
  uint32_t index() const { return index_; }

  QueueIndex Clear() const { return QueueIndex(0, count_); }

  // Returns a string representing the index.
  ::std::string DebugString() const;

 private:
  QueueIndex(uint32_t index, uint32_t count) : index_(index), count_(count) {}

  static constexpr uint32_t sentinal_value() { return 0xffffffffu; }

  friend struct AtomicQueueIndex;
  friend class Index;
  // For testing.
  friend class testing::QueueIndexTest;

  // Index and number of elements in the queue.
  uint32_t index_;
  // Count is stored here rather than passed in everywhere in the hopes that the
  // compiler completely optimizes out this class and this variable if it isn't
  // used.
  uint32_t count_;
};

// Atomic storage for setting and getting QueueIndex objects.
// Count is the number of messages in the queue.
struct AtomicQueueIndex {
 public:
  // Atomically reads the index without any ordering constraints.
  QueueIndex RelaxedLoad(uint32_t count) {
    return QueueIndex(index_.load(::std::memory_order_relaxed), count);
  }

  // Full bidirectional barriers here.
  QueueIndex Load(uint32_t count) {
    return QueueIndex(index_.load(::std::memory_order_acquire), count);
  }
  inline void Store(QueueIndex value) {
    index_.store(value.index_, ::std::memory_order_release);
  }

  // Invalidates the element unconditionally.
  inline void Invalidate() { Store(QueueIndex::Invalid()); }

  // Swaps expected for index atomically.  Returns true on success, false
  // otherwise.
  inline bool CompareAndExchangeStrong(QueueIndex expected, QueueIndex index) {
    return index_.compare_exchange_strong(expected.index_, index.index_,
                                          ::std::memory_order_acq_rel);
  }

 private:
  ::std::atomic<uint32_t> index_;
};

// Structure holding the queue index and the index into the message list.
class Index {
 public:
  // Constructs an Index.  queue_index is the QueueIndex of this message, and
  // message_index is the index into the messages structure.
  Index(QueueIndex queue_index, uint16_t message_index)
      : Index(queue_index.index_, message_index) {}
  Index(uint32_t queue_index, uint16_t message_index)
      : index_((queue_index & 0xffff) |
               (static_cast<uint32_t>(message_index) << 16)) {}

  // Index of this message in the message array.
  uint16_t message_index() const { return (index_ >> 16) & 0xffff; }

  // Lowest 16 bits of the queue index of this message in the queue.
  uint16_t queue_index() const { return index_ & 0xffff; }

  // Returns true if the provided queue index plausibly represents this Index.
  bool IsPlausible(QueueIndex queue_index) const {
    return queue_index.IsPlausible(this->queue_index());
  }

  // Returns an invalid Index.
  static Index Invalid() { return Index(sentinal_value()); }
  // Checks if this Index is valid or not.
  bool valid() const { return index_ != sentinal_value(); }

  // Returns the raw Index.  This should only be used for debug.
  uint32_t get() const { return index_; }

  // Returns the maximum number of messages we can store before overflowing.
  static constexpr uint16_t MaxMessages() { return 0xfffe; }

  bool operator==(const Index other) const { return other.index_ == index_; }

  // Returns a string representing the index.
  ::std::string DebugString() const;

 private:
  Index(uint32_t index)
      : index_(index) {}

  friend class AtomicIndex;

  static constexpr uint32_t sentinal_value() { return 0xffffffffu; }

  // Note: a value of 0xffffffff is a sentinal to represent an invalid entry.
  // This works because we would need to have a queue index of 0x*ffff, *and*
  // have 0xffff messages in the message list.  That constraint is easy to
  // enforce by limiting the max messages.
  uint32_t index_;
};

// Atomic storage for setting and getting Index objects.
class AtomicIndex {
 public:
  // Stores and loads atomically without ordering constraints.
  Index RelaxedLoad() {
    return Index(index_.load(::std::memory_order_relaxed));
  }
  void RelaxedStore(Index index) {
    index_.store(index.index_, ::std::memory_order_relaxed);
  }

  // Invalidates the index atomically, but without any ordering constraints.
  void RelaxedInvalidate() { RelaxedStore(Index::Invalid()); }

  // Full barriers here.
  void Invalidate() { Store(Index::Invalid()); }
  void Store(Index index) {
    index_.store(index.index_, ::std::memory_order_release);
  }
  Index Load() { return Index(index_.load(::std::memory_order_acquire)); }

  // Swaps expected for index atomically.  Returns true on success, false
  // otherwise.
  inline bool CompareAndExchangeStrong(Index expected, Index index) {
    return index_.compare_exchange_strong(expected.index_, index.index_,
                                          ::std::memory_order_acq_rel);
  }

 private:
  ::std::atomic<uint32_t> index_;
};

}  // namespace ipc_lib
}  // namespace aos

#endif  // AOS_IPC_LIB_INDEX_H_
