#ifndef AOS_EVENTS_LOGGING_CRC32_H_
#define AOS_EVENTS_LOGGING_CRC32_H_

#include <stdint.h>

#include <optional>

#include "absl/types/span.h"

namespace aos {

uint32_t ComputeCrc32(const absl::Span<uint8_t> data);

uint32_t AccumulateCrc32(
    const absl::Span<uint8_t> data,
    std::optional<uint32_t> current_checksum = std::nullopt);

}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_CRC32_H_
