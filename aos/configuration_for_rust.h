#ifndef AOS_CONFIGURATION_FOR_RUST_H_
#define AOS_CONFIGURATION_FOR_RUST_H_

#include <optional>

#include "aos/configuration.h"
#include "aos/for_rust.h"
#include "cxx.h"

namespace aos::configuration {

const Channel *GetChannelForRust(const Configuration *config, rust::Str name,
                                 rust::Str type, rust::Str application_name,
                                 const Node *node);

const Node *GetNodeForRust(const Configuration *config, rust::Str name);

// Returns a Configuration flatbuffer. Returns an empty vector on errors.
// TODO(Brian): It would be nice to return more detailed errors (not found vs
// could not parse vs merging error).
rust::Vec<uint8_t> MaybeReadConfigForRust(
    rust::Str path, rust::Slice<const rust::Str> extra_import_paths);

inline bool HasChannelTypeForRust(const Channel &channel) {
  return channel.type();
}
inline rust::Str GetChannelTypeForRust(const Channel &channel) {
  return StringViewToRustStr(channel.type()->string_view());
}

inline bool HasChannelNameForRust(const Channel &channel) {
  return channel.name();
}
inline rust::Str GetChannelNameForRust(const Channel &channel) {
  return StringViewToRustStr(channel.name()->string_view());
}

}  // namespace aos::configuration

#endif  // AOS_CONFIGURATION_FOR_RUST_H_
