#include "aos/init_for_rust.h"

#include "absl/flags/flag.h"
#include "absl/flags/reflection.h"
#include "absl/log/check.h"
#include "absl/log/initialize.h"
#include "absl/log/log.h"

#include "aos/init.h"

namespace aos {

void InitFromRust(const char * /*argv0*/) {
  CHECK(!aos::IsInitialized()) << "Only initialize once.";

  absl::InitializeLog();

  // TODO(Brian): Where should Rust binaries be configured to write coredumps?

  // TODO(Brian): Figure out what to do with allocator hooks for C++ and Rust.

  MarkInitialized();
}

std::vector<FlagInfo> GetCppFlags() {
  absl::flat_hash_map<absl::string_view, absl::CommandLineFlag *> info =
      absl::GetAllFlags();
  std::vector<FlagInfo> out;
  for (const auto &flag : info) {
    std::string type;
    if (flag.second->IsOfType<float>()) {
      type = "float";
    } else if (flag.second->IsOfType<double>()) {
      type = "double";
    } else if (flag.second->IsOfType<bool>()) {
      type = "bool";
    } else if (flag.second->IsOfType<uint8_t>()) {
      type = "uint8_t";
    } else if (flag.second->IsOfType<int8_t>()) {
      type = "int8_t";
    } else if (flag.second->IsOfType<uint16_t>()) {
      type = "uint16_t";
    } else if (flag.second->IsOfType<int16_t>()) {
      type = "int16_t";
    } else if (flag.second->IsOfType<uint32_t>()) {
      type = "uint32_t";
    } else if (flag.second->IsOfType<int32_t>()) {
      type = "int32_t";
    } else if (flag.second->IsOfType<uint64_t>()) {
      type = "uint64_t";
    } else if (flag.second->IsOfType<int64_t>()) {
      type = "int64_t";
    } else if (flag.second->IsOfType<std::string>()) {
      type = "string";
    } else if (flag.second->IsOfType<std::vector<std::string>>()) {
      type = "vector<string>";
    } else {
      LOG(FATAL) << "Unknown type for flag " << flag.second->Name()
                 << " in file " << flag.second->Filename();
    }

    LOG(INFO) << "Reporting flag " << flag.second->Name() << " " << type << " "
              << flag.second->DefaultValue();
    FlagInfo out_flag = {
        .name_ = std::string(flag.second->Name()),
        .type_ = type,
        .description_ = flag.second->Help(),
        .default_value_ = flag.second->DefaultValue(),
        .filename_ = flag.second->Filename(),
    };
    out.push_back(out_flag);
  }
  return out;
}

bool SetCommandLineOption(const char *name, const char *value) {
  absl::CommandLineFlag *flag = absl::FindCommandLineFlag(name);
  if (flag == nullptr) {
    return false;
  }

  std::string error;
  bool result = flag->ParseFrom(value, &error);
  return result;
}

std::string GetCommandLineOption(const char *name) {
  absl::CommandLineFlag *flag = absl::FindCommandLineFlag(name);
  CHECK(flag != nullptr) << "Unknown flag " << name;

  return flag->CurrentValue();
}

}  // namespace aos
