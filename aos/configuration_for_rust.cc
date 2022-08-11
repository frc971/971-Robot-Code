#include "aos/configuration_for_rust.h"

#include "aos/for_rust.h"

namespace aos::configuration {

const Channel *GetChannelForRust(const Configuration *config, rust::Str name,
                                 rust::Str type, rust::Str application_name,
                                 const Node *node) {
  return GetChannel(config, RustStrToStringView(name),
                    RustStrToStringView(type),
                    RustStrToStringView(application_name), node);
}

rust::Vec<uint8_t> MaybeReadConfigForRust(
    rust::Str path, rust::Slice<const rust::Str> extra_import_paths) {
  std::vector<std::string_view> cc_extra_import_paths;
  for (const rust::Str &extra_import_path : extra_import_paths) {
    cc_extra_import_paths.push_back(RustStrToStringView(extra_import_path));
  }
  const auto cc_result = MaybeReadConfig(RustStrToStringView(path));
  rust::Vec<uint8_t> result;
  if (cc_result) {
    result.reserve(cc_result->span().size());
    for (uint8_t b : cc_result->span()) {
      result.push_back(b);
    }
  }
  return result;
}

}  // namespace aos::configuration
