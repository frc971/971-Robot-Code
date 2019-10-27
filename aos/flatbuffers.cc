#include "aos/flatbuffers.h"

#include "glog/logging.h"

namespace aos {

uint8_t *FixedAllocatorBase::allocate(size_t) {
  if (is_allocated_) {
    LOG(FATAL) << "Tried to allocate already allocated flatbuffer";
  }

  is_allocated_ = true;
  return data();
}

uint8_t *FixedAllocatorBase::reallocate_downward(uint8_t *, size_t, size_t,
                                                 size_t, size_t) {
  LOG(FATAL) << "Tried to reallocate a flatbuffer";
  return nullptr;
}

}  // namespace aos
