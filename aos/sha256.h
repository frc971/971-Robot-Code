#ifndef AOS_SHA256_H_
#define AOS_SHA256_H_

#include <filesystem>
#include <string>

#include "absl/types/span.h"

namespace aos {

// Returns the sha256 of a span.
std::string Sha256(const absl::Span<const uint8_t> str);
std::string Sha256(std::string_view str);

// Returns the Sha256 of the specified file. Dies on failure to read the file.
std::string Sha256OfFile(std::filesystem::path file);

}  // namespace aos

#endif  // AOS_SHA256_H_
