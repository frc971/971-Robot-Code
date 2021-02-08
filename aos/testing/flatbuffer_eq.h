#ifndef AOS_TESTING_FLATBUFFER_EQ_H_
#define AOS_TESTING_FLATBUFFER_EQ_H_

#include "aos/flatbuffer_merge.h"
#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "gmock/gmock.h"

namespace aos {
namespace testing {

// Use FlatbufferEq to instantiate this.
template <typename T>
class FlatbufferEqMatcher {
 public:
  FlatbufferEqMatcher(aos::FlatbufferString<T> expected)
      : expected_(std::move(expected)) {}

  bool MatchAndExplain(const T *t,
                       ::testing::MatchResultListener *listener) const {
    *listener << "is " << aos::FlatbufferToJson(t);
    return aos::CompareFlatBuffer(t, &expected_.message());
  }

  bool MatchAndExplain(const aos::Flatbuffer<T> &t,
                       ::testing::MatchResultListener *listener) const {
    return MatchAndExplain(&t.message(), listener);
  }

  void DescribeTo(std::ostream *os) const {
    *os << "is equal to " << aos::FlatbufferToJson(&expected_.message());
  }

  void DescribeNegationTo(std::ostream *os) const {
    *os << "is not equal to " << aos::FlatbufferToJson(&expected_.message());
  }

 private:
  const aos::FlatbufferString<T> expected_;
};

// Returns a googlemock matcher which will compare a `const T *` or a `const
// aos::Flatbuffer<T> &` against expected. This will automatically give nice
// error messages if they don't match.
//
// T must be a flatbuffer table type.
template <typename T>
::testing::PolymorphicMatcher<FlatbufferEqMatcher<T>> FlatbufferEq(
    const aos::NonSizePrefixedFlatbuffer<T> &expected) {
  return ::testing::MakePolymorphicMatcher(
      FlatbufferEqMatcher(aos::FlatbufferString<T>(expected)));
}

}  // namespace testing
}  // namespace aos

#endif  // AOS_TESTING_FLATBUFFER_EQ_H_
