#ifndef AOS_EVENTS_CHANNEL_PREALLOCATED_ALLOCATOR_
#define AOS_EVENTS_CHANNEL_PREALLOCATED_ALLOCATOR_

#include "aos/configuration.h"
#include "aos/configuration_generated.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {

class ChannelPreallocatedAllocator : public flatbuffers::Allocator {
 public:
  ChannelPreallocatedAllocator(uint8_t *data, size_t size,
                               const Channel *channel)
      : data_(data), size_(size), channel_(channel) {}

  ChannelPreallocatedAllocator(const ChannelPreallocatedAllocator &) = delete;
  ChannelPreallocatedAllocator(ChannelPreallocatedAllocator &&other)
      : data_(other.data_), size_(other.size_), channel_(other.channel_) {
    CHECK(!is_allocated()) << ": May not overwrite in-use allocator";
    CHECK(!other.is_allocated());
  }

  ChannelPreallocatedAllocator &operator=(
      const ChannelPreallocatedAllocator &) = delete;
  ChannelPreallocatedAllocator &operator=(
      ChannelPreallocatedAllocator &&other) {
    CHECK(!is_allocated()) << ": May not overwrite in-use allocator";
    CHECK(!other.is_allocated());
    data_ = other.data_;
    size_ = other.size_;
    channel_ = other.channel_;
    return *this;
  }
  ~ChannelPreallocatedAllocator() override { CHECK(!is_allocated_); }

  // TODO(austin): Read the contract for these.
  uint8_t *allocate(size_t /*size*/) override {
    if (is_allocated_) {
      LOG(FATAL) << "Can't allocate more memory with a fixed size allocator.  "
                    "Increase the memory reserved.";
    }

    is_allocated_ = true;
    return data_;
  }

  void deallocate(uint8_t *, size_t) override { is_allocated_ = false; }

  uint8_t *reallocate_downward(uint8_t * /*old_p*/, size_t /*old_size*/,
                               size_t new_size, size_t /*in_use_back*/,
                               size_t /*in_use_front*/) override {
    LOG(FATAL) << "Requested " << new_size << " bytes, max size "
               << channel_->max_size() << " for channel "
               << configuration::CleanedChannelToString(channel_)
               << ".  Increase the memory reserved to at least " << new_size
               << ".";
    return nullptr;
  }

  void Reset() { is_allocated_ = false; }
  bool is_allocated() const { return is_allocated_; }

  bool allocated() { return is_allocated_; }

  size_t size() const { return size_; }
  const uint8_t *data() const { return data_; }

 private:
  bool is_allocated_ = false;
  uint8_t *data_;
  size_t size_;
  const Channel *channel_;
};

}  // namespace aos

#endif  // AOS_EVENTS_CHANNEL_PREALLOCATED_ALLOCATOR_
