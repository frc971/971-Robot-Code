#ifndef _AOS_VISION_REGION_ALLOC_H_
#define _AOS_VISION_REGION_ALLOC_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <new>
#include <utility>
#include <vector>

#include "aos/common/logging/logging.h"

namespace aos {
namespace vision {

// Region based allocator. Used for arena allocations in vision code.
// Example use: Storing contour nodes.
class AnalysisAllocator {
 public:
  template <typename T, typename... Args>
  T *cons_obj(Args &&... args) {
    uint8_t *ptr = NULL;
    if (sizeof(T) + alignof(T) > block_size_) {
      LOG(FATAL, "allocating %d too much\n", (int)sizeof(T));
    }
    while (ptr == NULL) {
      if (next_free_ >= memory_.size()) {
        if (next_free_ >= 1024) {
          LOG(FATAL, "too much alloc\n");
        }
        memory_.emplace_back(new uint8_t[block_size_]);
      } else if ((used_size_ % alignof(T)) != 0) {
        used_size_ += alignof(T) - (used_size_ % alignof(T));
      } else if ((used_size_ + sizeof(T)) <= block_size_) {
        ptr = &memory_[next_free_][used_size_];
        used_size_ += sizeof(T);
      } else {
        used_size_ = 0;
        next_free_++;
      }
    }
    return new (ptr) T(std::forward<Args>(args)...);
  }
  void reset() {
    next_free_ = 0;
    used_size_ = 0;
  }

 private:
  std::vector<std::unique_ptr<uint8_t[]>> memory_;
  size_t next_free_ = 0;
  size_t block_size_ = 1024 * 4;
  size_t used_size_ = 0;
};

}  // namespace vision
}  // namespace aos

#endif  // _AOS_VISION_IMAGE_REGION_ALLOC_H_
