#ifndef AOS_TESTING_PATH_H_
#define AOS_TESTING_PATH_H_

#include <string>
#include <string_view>

namespace aos::testing {

// Returns the path to the provided artifact which works
std::string ArtifactPath(std::string_view path);

}  // namespace aos::testing

#endif  // AOS_TESTING_PATH_H_
