#include "aos/flatbuffers.h"

#include "glog/logging.h"

namespace aos {

uint8_t *FixedAllocatorBase::allocate(size_t) {
  if (is_allocated_) {
    LOG(FATAL) << "Can't allocate more memory with a fixed size allocator.  "
                  "Increase the memory reserved.";
  }

  is_allocated_ = true;
  return data();
}

uint8_t *FixedAllocatorBase::reallocate_downward(uint8_t *, size_t, size_t,
                                                 size_t, size_t) {
  LOG(FATAL) << "Can't allocate more memory with a fixed size allocator.  "
                "Increase the memory reserved.";
  return nullptr;
}

}  // namespace aos
