#ifndef AOS_SHA256_H_
#define AOS_SHA256_H_

#include <string>

#include "absl/types/span.h"

namespace aos {

// Returns the sha256 of a span.
std::string Sha256(const absl::Span<const uint8_t> str);

}  // namespace aos

#endif  // AOS_SHA256_H_
