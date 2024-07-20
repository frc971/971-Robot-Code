#include "aos/util/status.h"

#include "absl/log/log.h"

#include "aos/containers/inlined_vector.h"

namespace aos {
namespace {
// Constructs a string view from the provided buffer if it has data and
// otherwise uses the provided string view. Used in copy/move constructors to
// figure out whether we should use the buffer or keep the pointer to the
// existing std::string_view (as is the case for when we store a pointer to a
// string literal).
static std::string_view MakeStringViewFromBufferOrView(
    const aos::InlinedVector<char, Error::kStaticMessageLength> &buffer,
    const std::string_view &view) {
  return (buffer.size() > 0) ? std::string_view(buffer.begin(), buffer.end())
                             : view;
}
}  // namespace
Error::Error(StatusCode code, std::string_view message,
             std::optional<std::source_location> source_location)
    : code_(code),
      owned_message_(message.begin(), message.end()),
      message_(owned_message_.data(), owned_message_.size()),
      source_location_(std::move(source_location)) {}
Error::Error(StatusCode code, const char *message,
             std::optional<std::source_location> source_location)
    : code_(code),
      message_(message),
      source_location_(std::move(source_location)) {}

Error::Error(Error &&other)
    : code_(other.code_),
      owned_message_(std::move(other.owned_message_)),
      message_(MakeStringViewFromBufferOrView(owned_message_, other.message_)),
      source_location_(std::move(other.source_location_)) {
  // Because the internal string view contains a pointer to the owned_message_
  // buffer, we need to have a manually written move constructor to manage it.
  other.message_ = {};
}
Error &Error::operator=(Error &&other) {
  std::swap(*this, other);
  return *this;
}
Error::Error(const Error &other)
    : code_(other.code_),
      owned_message_(other.owned_message_),
      message_(MakeStringViewFromBufferOrView(owned_message_, other.message_)),
      source_location_(other.source_location_) {}

std::string Error::ToString() const {
  std::string source_info = "";
  if (source_location_.has_value()) {
    source_info = absl::StrFormat(
        "%s:%d in %s: ", source_location_->file_name(),
        source_location_->line(), source_location_->function_name());
  }

  return absl::StrFormat("%sErrored with code of %d and message: %s",
                         source_info, code(), message());
}

template <>
void CheckExpected<void>(const Result<void> &expected) {
  if (expected.has_value()) {
    return;
  }
  LOG(FATAL) << expected.error().ToString();
}

int ResultExitCode(const Result<void> &expected) {
  return expected.has_value() ? static_cast<int>(Error::StatusCode::kOk)
                              : expected.error().code();
}
}  // namespace aos
