#include "aos/testing/path.h"

#include "absl/strings/str_cat.h"

namespace aos::testing {

// Returns the path to the provided artifact which works when built both as an
// external target and in the repo.
std::string ArtifactPath(std::string_view path) {
  return absl::StrCat("../" AOS_REPO_NAME "/", path);
}

}  // namespace aos::testing
