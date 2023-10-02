#include "aos/init_for_rust.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "aos/init.h"

namespace aos {

void InitFromRust(const char *argv0) {
  CHECK(!IsInitialized()) << "Only initialize once.";

  google::InitGoogleLogging(argv0);

  // TODO(Brian): Where should Rust binaries be configured to write coredumps?

  // TODO(Brian): Figure out what to do with allocator hooks for C++ and Rust.

  MarkInitialized();
}

std::vector<FlagInfo> GetCppFlags() {
  std::vector<gflags::CommandLineFlagInfo> info;
  gflags::GetAllFlags(&info);
  std::vector<FlagInfo> out;
  for (const auto &flag : info) {
    FlagInfo out_flag = {
        .name_ = flag.name,
        .type_ = flag.type,
        .description_ = flag.description,
        .default_value_ = flag.default_value,
        .filename_ = flag.filename,
    };
    out.push_back(out_flag);
  }
  return out;
}

bool SetCommandLineOption(const char *name, const char *value) {
  return !gflags::SetCommandLineOption(name, value).empty();
}

std::string GetCommandLineOption(const char *name) {
  std::string out;
  CHECK(gflags::GetCommandLineOption(name, &out)) << "Unknown flag " << name;
  return out;
}

}  // namespace aos
