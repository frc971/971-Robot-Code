#ifndef AOS_FOR_RUST_H_
#define AOS_FOR_RUST_H_

// This file has some shared utilities for the for_rust C++ code, which provides
// autocxx-friendly versions of C++ APIs.

#include <string_view>

#include "cxx.h"

namespace aos {

inline rust::Str StringViewToRustStr(std::string_view s) {
  return rust::Str(s.data(), s.size());
}

inline std::string_view RustStrToStringView(rust::Str s) {
  return std::string_view(s.data(), s.size());
}

}  // namespace aos

#endif  // AOS_FOR_RUST_H_
