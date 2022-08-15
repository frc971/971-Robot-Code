#ifndef AOS_CONFIGURATION_FOR_RUST_H_
#define AOS_CONFIGURATION_FOR_RUST_H_

#include "aos/configuration.h"
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

}  // namespace aos::configuration

#endif  // AOS_CONFIGURATION_FOR_RUST_H_
