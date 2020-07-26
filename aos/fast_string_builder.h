#ifndef AOS_FAST_STRING_BUILDER_H_
#define AOS_FAST_STRING_BUILDER_H_

#include <ostream>
#include <string>
#include <type_traits>

#include "absl/strings/numbers.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"

namespace aos {

// Simplified string builder which is faster than standard classes
// (std::string::append(), std::ostringstream). Reduces copies on insertion
// and retrieval, uses fast number to string conversions, and allows reuse
// of the internal buffer.
class FastStringBuilder {
 public:
  FastStringBuilder(std::size_t initial_size = 64);

  // Convert bools from integer representation to "true" and "false".
  // Defaults on.
  void set_bool_to_str(bool bool_to_str) { bool_to_str_ = bool_to_str; }

  // Clears result, allows for reuse of builder without reallocation.
  // Invalidates Result()
  void Reset();

  // Returns a temporary view to the string result. Invalidated on destruction
  // of builder, after calling Reset(), or after calling MoveResult().
  std::string_view Result() const;

  // Release the string held by the builder. Resets builder and invalidates
  // Result().
  std::string MoveResult();

  // Append integer to result, converted to string representation.
  template <typename T, typename = std::enable_if_t<std::is_integral_v<T>>>
  void AppendInt(T val);

  void Append(std::string_view);

  void Append(const char *c) { Append(std::string_view(c)); }

  // Append character to result as character.
  void AppendChar(char);

  // Append float or double to result, converted to string representation
  void Append(float);

  void Append(double);

  // Append bool to result. set_bool_to_str() toggles between integer and string
  // representation.
  void AppendBool(bool);

  // Prints the current result to output
  friend std::ostream &operator<<(std::ostream &os,
                                  const FastStringBuilder &builder);

 private:
  static const inline std::string kTrue = "true", kFalse = "false";

  // Allocate additional space of at least chars_needed relative to index_.
  void Resize(std::size_t chars_needed);

  std::string str_;
  bool bool_to_str_ = true;
};

template <typename T, typename>
void FastStringBuilder::AppendInt(T val) {
  std::size_t index = str_.size();
  Resize(absl::numbers_internal::kFastToBufferSize);
  char *end = absl::numbers_internal::FastIntToBuffer(val, str_.data() + index);
  str_.resize(end - str_.data());
}

}  // namespace aos

#endif  // AOS_FAST_STRING_BUIDLER_H_
