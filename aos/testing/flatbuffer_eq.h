#ifndef AOS_TESTING_FLATBUFFER_EQ_H_
#define AOS_TESTING_FLATBUFFER_EQ_H_

#include "aos/flatbuffer_merge.h"
#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "gmock/gmock.h"

namespace aos {
namespace testing {

// Use FlatbufferUnwrapped to instantiate this.
template <typename T>
class FlatbufferUnwrappedMatcher {
 public:
  FlatbufferUnwrappedMatcher(::testing::Matcher<const T *> matcher)
      : matcher_(std::move(matcher)) {}

  bool MatchAndExplain(const T *t,
                       ::testing::MatchResultListener *listener) const {
    return matcher_.MatchAndExplain(t, listener);
  }

  bool MatchAndExplain(const aos::Flatbuffer<T> &t,
                       ::testing::MatchResultListener *listener) const {
    return MatchAndExplain(&t.message(), listener);
  }

  void DescribeTo(std::ostream *os) const { matcher_.DescribeTo(os); }

  void DescribeNegationTo(std::ostream *os) const {
    matcher_.DescribeNegationTo(os);
  }

 private:
  const ::testing::Matcher<const T *> matcher_;
};

// Returns a googlemock matcher which will compare a `const T *` or a `const
// aos::Flatbuffer<T> &` against another matcher which only handles `const T *`.
// This will automatically propagate the nice error messages.
//
// T must be a flatbuffer table type.
template <typename T>
inline auto FlatbufferUnwrapped(::testing::Matcher<const T *> matcher) {
  return ::testing::MakePolymorphicMatcher(
      FlatbufferUnwrappedMatcher(std::move(matcher)));
}

// Use FlatbufferEq to instantiate this.
template <typename T>
class FlatbufferEqMatcher : public ::testing::MatcherInterface<const T *> {
 public:
  FlatbufferEqMatcher(aos::FlatbufferString<T> expected)
      : expected_(std::move(expected)) {}
  ~FlatbufferEqMatcher() override = default;

  bool MatchAndExplain(
      const T *t, ::testing::MatchResultListener *listener) const override {
    *listener << "is " << aos::FlatbufferToJson(t);
    return aos::CompareFlatBuffer(t, &expected_.message());
  }

  void DescribeTo(std::ostream *os) const override {
    *os << "is equal to " << aos::FlatbufferToJson(&expected_.message());
  }

  void DescribeNegationTo(std::ostream *os) const override {
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
inline auto FlatbufferEq(const aos::NonSizePrefixedFlatbuffer<T> &expected) {
  return FlatbufferUnwrapped(::testing::MakeMatcher(
      new FlatbufferEqMatcher(aos::FlatbufferString<T>(expected))));
}

}  // namespace testing
}  // namespace aos

#endif  // AOS_TESTING_FLATBUFFER_EQ_H_
