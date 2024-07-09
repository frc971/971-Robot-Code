#ifndef AOS_SHARED_SPAN_H_
#define AOS_SHARED_SPAN_H_

#include <cstdint>
#include <memory>

#include "absl/types/span.h"

namespace aos {

// Shared pointer to a region of memory.  The pointer needs to own the region.
using SharedSpan = std::shared_ptr<const absl::Span<const uint8_t>>;

}  // namespace aos

#endif  // AOS_SHARED_SPAN_H_
