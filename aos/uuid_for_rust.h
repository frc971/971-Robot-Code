#ifndef AOS_UUID_FOR_RUST_H_
#define AOS_UUID_FOR_RUST_H_

#include "aos/uuid.h"

namespace aos {

// An alternative version of UUID to feed autocxx, to work around
// https://github.com/google/autocxx/issues/266.
/// <div rustbindgen replaces="aos::UUID"></div>
struct RustUUID {
  uint8_t data[16];
};

static_assert(sizeof(UUID) == sizeof(RustUUID));
static_assert(alignof(UUID) == alignof(RustUUID));

}  // namespace aos

#endif  // AOS_UUID_FOR_RUST_H_
