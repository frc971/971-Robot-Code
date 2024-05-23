#include "aos/util/status.h"

namespace aos {
Status::Status(StatusCode code, std::string_view message,
               std::optional<std::source_location> source_location)
    : code_(code),
      owned_message_(message.begin(), message.end()),
      message_(owned_message_.data(), owned_message_.size()),
      source_location_(std::move(source_location)) {}
Status::Status(StatusCode code, const char *message,
               std::optional<std::source_location> source_location)
    : code_(code),
      message_(message),
      source_location_(std::move(source_location)) {}

Status::Status(Status &&other)
    : code_(other.code_),
      owned_message_(std::move(other.owned_message_)),
      message_(MakeStringViewFromBufferOrView(owned_message_, other.message_)),
      source_location_(std::move(other.source_location_)) {
  // Because the internal string view contains a pointer to the owned_message_
  // buffer, we need to have a manually written move constructor to manage it.
  other.message_ = {};
}
Status &Status::operator=(Status &&other) {
  std::swap(*this, other);
  return *this;
}
Status::Status(const Status &other)
    : code_(other.code_),
      owned_message_(other.owned_message_),
      message_(MakeStringViewFromBufferOrView(owned_message_, other.message_)),
      source_location_(other.source_location_) {}

std::string Status::ToString() const {
  std::string source_info = "";
  if (source_location_.has_value()) {
    source_info = absl::StrFormat(
        "%s:%d in %s: ", source_location_->file_name(),
        source_location_->line(), source_location_->function_name());
  }

  return absl::StrFormat("%sStatus is %s with code of %d and message: %s",
                         source_info, ok() ? "okay" : "errored", code(),
                         message());
}

template <>
void CheckExpected<void>(const tl::expected<void, Status> &expected) {
  if (expected.has_value()) {
    return;
  }
  LOG(FATAL) << expected.error().ToString();
}
}  // namespace aos
