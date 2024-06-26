#include "aos/ipc_lib/shm_base.h"

#include <string>

#include "absl/flags/flag.h"

ABSL_FLAG(std::string, shm_base, "/dev/shm/aos",
          "Directory to place queue backing mmaped files in.");

namespace aos::testing {

void SetShmBase(const std::string_view base) {
  absl::SetFlag(&FLAGS_shm_base, std::string(base) + "/aos");
}

}  // namespace aos::testing
