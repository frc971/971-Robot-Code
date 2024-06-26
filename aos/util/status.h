#ifndef AOS_UTIL_STATUS_H_
#define AOS_UTIL_STATUS_H_
#include <optional>
#include <source_location>
#include <string_view>

#include "absl/log/log.h"
#include "absl/strings/str_format.h"
#include "tl/expected.hpp"

#include "aos/containers/inlined_vector.h"

namespace aos {
// The Error class provides a means by which errors can be readily returned
// from methods. It will typically be wrapped by an std::expected<> to
// accommodate a return value or the Error.
//
// The Error class is similar to the absl::Status or std::error_code classes,
// in that it consists of an integer error code of some sort (where 0 implicitly
// would indicate "ok", although we assume that if there is no error then
// you will be using an expected<> to return void or your actual return type)
// and a string error message of some sort. The main additions of this
// class are:
// 1. Adding a first-class exposure of an std::source_location to make exposure
//    of the sources of errors easier.
// 2. Providing an interface that allows for Error implementations that expose
//    messages without malloc'ing (not possible with absl::Status, although it
//    is possible with std::error_code).
// 3. Making it relatively easy to quickly return a simple error & message
//    (specifying a custom error with std::error_code is possible but requires
//    jumping through hoops and managing some global state).
// 4. Does not support an "okay" state, to make it clear that the user is
//    supposed to use a wrapper that will itself indicate okay.
//
// The goal of this class is that it should be easy to convert from existing
// error types (absl::Status, std::error_code) to this type.
//
// Users should typically use the Result<T> convenience method when returning
// Errors from methods. In the case where the method would normally return void,
// use Result<void>. Result<> is just a wrapper for tl::expected; when our
// compilers upgrade to support std::expected this should ease the transition,
// in addition to just providing a convenience wrapper to encourage a standard
// pattern of use.
class Error {
 public:
  // In order to allow simple error messages without memory allocation, we
  // reserve a small amount of stack space for error messages. This constant
  // specifies the length of these strings.
  static constexpr size_t kStaticMessageLength = 128;

  // Attaches human-readable status enums to integer codes---the specific
  // numeric codes are used as exit codes when terminating execution of the
  // program.
  // Note: While 0 will always indicate success and non-zero values will always
  // indicate failures we may attempt to further expand the set of non-zero exit
  // codes in the future and may decide to reuse 1 for a more specific error
  // code at the time (although it is reasonably likely that it will be kept as
  // a catch-all general error).
  enum class StatusCode : int {
    kOk = 0,
    kError = 1,
  };

  // Constructs an Error, copying the provided message. If the message is
  // shorter than kStaticMessageLength, then the message will be stored entirely
  // on the stack; longer messages will require dynamic memory allocation.
  // The default source_location will correspond to the call-site of the
  // Error::Error() method. This should only be overridden by wrappers that
  // want to present a fancier interface to users.
  static Error MakeError(
      std::string_view message,
      std::source_location source_location = std::source_location::current()) {
    return Error(StatusCode::kError, message, std::move(source_location));
  }
  static tl::unexpected<Error> MakeUnexpectedError(
      std::string_view message,
      std::source_location source_location = std::source_location::current()) {
    return tl::unexpected<Error>(
        MakeError(message, std::move(source_location)));
  }

  // Constructs an error, retaining the provided pointer to a null-terminated
  // error message. It is assumed that the message pointer will stay valid
  // ~indefinitely. This is generally only appropriate to use with string
  // literals (e.g., Error::StringLiteralError("Hello, World!")).
  // The default source_location will correspond to the call-site of the
  // Error::Error() method. This should only be overridden by wrappers that
  // want to present a fancier interface to users.
  static Error MakeStringLiteralError(
      const char *message,
      std::source_location source_location = std::source_location::current()) {
    return Error(StatusCode::kError, message, std::move(source_location));
  }
  static tl::unexpected<Error> MakeUnexpectedStringLiteralError(
      const char *message,
      std::source_location source_location = std::source_location::current()) {
    return tl::unexpected<Error>(
        MakeStringLiteralError(message, std::move(source_location)));
  }

  Error(Error &&other);
  Error &operator=(Error &&other);
  Error(const Error &other);

  // Returns a numeric value for the status code. Zero will always indicate
  // success; non-zero values will always indicate an error.
  [[nodiscard]] int code() const { return static_cast<int>(code_); }
  // Returns a view of the error message.
  [[nodiscard]] std::string_view message() const { return message_; }
  // Returns the source_location attached to the current Error. If the
  // source_location was never set, will return nullopt. The source_location
  // will typically be left unset for successful ("ok") statuses.
  [[nodiscard]] const std::optional<std::source_location> &source_location()
      const {
    return source_location_;
  }

  std::string ToString() const;

 private:
  Error(StatusCode code, std::string_view message,
        std::optional<std::source_location> source_location);
  Error(StatusCode code, const char *message,
        std::optional<std::source_location> source_location);

  StatusCode code_;
  aos::InlinedVector<char, kStaticMessageLength> owned_message_;
  std::string_view message_;
  std::optional<std::source_location> source_location_;
};

template <typename T>
using Result = tl::expected<T, Error>;

// Dies fatally if the provided expected does not include the value T, printing
// out an error message that includes the Error on the way out.
// Returns the stored value on success.
template <typename T>
T CheckExpected(const Result<T> &expected) {
  if (expected.has_value()) {
    return expected.value();
  }
  LOG(FATAL) << expected.error().ToString();
}

template <>
void CheckExpected<void>(const Result<void> &expected);

int ResultExitCode(const Result<void> &expected);
}  // namespace aos
#endif  // AOS_UTIL_STATUS_H_
