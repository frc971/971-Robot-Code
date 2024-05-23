#ifndef AOS_UTIL_STATUS_H_
#define AOS_UTIL_STATUS_H_
#include <optional>
#include <source_location>
#include <string_view>

#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "tl/expected.hpp"

#include "aos/containers/inlined_vector.h"

namespace aos {
// The Status class provides a means by which errors can be readily returned
// from methods. It will typically be wrapped by an std::expected<> to
// accommodate a return value or the Status, although an "ok" status can also be
// used to indicate no-error.
//
// The Status class is similar to the absl::Status or std::error_code classes,
// in that it consists of an integer error code of some sort (where 0 indicates
// "ok") and a string error message of some sort. The main additions of this
// class are:
// 1. Adding a first-class exposure of an std::source_location to make exposure
//    of the sources of errors easier.
// 2. Providing an interface that allows for Status implementations that expose
//    messages without malloc'ing (not possible with absl::Status, although it
//    is possible with std::error_code).
// 3. Making it relatively easy to quickly return a simple error & message
//    (specifying a custom error with std::error_code is possible but requires
//    jumping through hoops and managing some global state).
//
// The goal of this class is that it should be easy to convert from exiting
// error types (absl::Status, std::error_code) to this type.
class Status {
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
  // Constructs a status that indicates success, with no associated error
  // message our source location.
  static Status Ok() { return Status(StatusCode::kOk, "", std::nullopt); }
  // Constructs an Error, copying the provided message. If the message is
  // shorter than kStaticMessageLength, then the message will be stored entirely
  // on the stack; longer messages will require dynamic memory allocation.
  // The default source_location will correspond to the call-site of the
  // Status::Error() method. This should only be overridden by wrappers that
  // want to present a fancier interface to users.
  static Status Error(
      std::string_view message,
      std::source_location source_location = std::source_location::current()) {
    return Status(StatusCode::kError, message, std::move(source_location));
  }
  static tl::unexpected<Status> UnexpectedError(
      std::string_view message,
      std::source_location source_location = std::source_location::current()) {
    return tl::unexpected<Status>(Error(message, std::move(source_location)));
  }
  // Constructs an error, retaining the provided pointer to a null-terminated
  // error message. It is assumed that the message pointer will stay valid
  // ~indefinitely. This is generally only appropriate to use with string
  // literals (e.g., Status::StringLiteralError("Hello, World!")).
  // The default source_location will correspond to the call-site of the
  // Status::Error() method. This should only be overridden by wrappers that
  // want to present a fancier interface to users.
  static Status StringLiteralError(
      const char *message,
      std::source_location source_location = std::source_location::current()) {
    return Status(StatusCode::kError, message, std::move(source_location));
  }
  static tl::unexpected<Status> UnexpectedStringLiteralError(
      const char *message,
      std::source_location source_location = std::source_location::current()) {
    return tl::unexpected<Status>(
        StringLiteralError(message, std::move(source_location)));
  }

  Status(Status &&other);
  Status &operator=(Status &&other);
  Status(const Status &other);

  // Returns true if the Status indicates success.
  [[nodiscard]] bool ok() const { return code_ == StatusCode::kOk; }
  // Returns a numeric value for the status code. Zero will always indicate
  // success; non-zero values will always indicate an error.
  [[nodiscard]] int code() const { return static_cast<int>(code_); }
  // Returns a view of the error message.
  [[nodiscard]] std::string_view message() const { return message_; }
  // Returns the source_location attached to the current Status. If the
  // source_location was never set, will return nullopt. The source_location
  // will typically be left unset for successful ("ok") statuses.
  [[nodiscard]] const std::optional<std::source_location> &source_location()
      const {
    return source_location_;
  }

  std::string ToString() const;

 private:
  Status(StatusCode code, std::string_view message,
         std::optional<std::source_location> source_location);
  Status(StatusCode code, const char *message,
         std::optional<std::source_location> source_location);

  // Constructs a string view from the provided buffer if it has data and
  // otherwise uses the provided string view. Used in copy/move constructors to
  // figure out whether we should use the buffer or keep the pointer to the
  // existing std::string_view (as is the case for when we store a pointer to a
  // string literal).
  static std::string_view MakeStringViewFromBufferOrView(
      const aos::InlinedVector<char, kStaticMessageLength> &buffer,
      const std::string_view &view) {
    return (buffer.size() > 0) ? std::string_view(buffer.begin(), buffer.end())
                               : view;
  }

  StatusCode code_;
  aos::InlinedVector<char, kStaticMessageLength> owned_message_;
  std::string_view message_;
  std::optional<std::source_location> source_location_;
};

// Dies fatally if the provided expected does not include the value T, printing
// out an error message that includes the Status on the way out.
// Returns the stored value on success.
template <typename T>
T CheckExpected(const tl::expected<T, Status> &expected) {
  if (expected.has_value()) {
    return expected.value();
  }
  LOG(FATAL) << expected.error().ToString();
}

template <>
void CheckExpected<void>(const tl::expected<void, Status> &expected);
}  // namespace aos
#endif  // AOS_UTIL_STATUS_H_
