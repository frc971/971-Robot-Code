#include "aos/testing/path.h"
#include "absl/strings/str_cat.h"

namespace aos {
namespace testing {

// Returns the path to the provided artifact which works when built both as an
// external target and in the repo.
std::string ArtifactPath(std::string_view path) {
  // TODO(austin): Don't hard-code the repo name here since it likely will
  // change.
  return absl::StrCat("../org_frc971/", path);
}

}  // namespace testing
}  // namespace aos
