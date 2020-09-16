#ifndef AOS_EVENTS_LOGGING_UUID_H_
#define AOS_EVENTS_LOGGING_UUID_H_

#include <array>
#include <random>
#include <string_view>

namespace aos {

// Class to generate and hold a UUID.
class UUID {
 public:
  // Returns a randomly generated UUID.  This is known as a UUID4.
  static UUID Random();

  std::string_view string_view() const {
    return std::string_view(data_.data(), data_.size());
  }

  bool operator==(const UUID &other) const {
    return other.string_view() == string_view();
  }
  bool operator!=(const UUID &other) const {
    return other.string_view() != string_view();
  }

 private:
  UUID() {}

  // Fixed size storage for the data.  Non-null terminated.
  std::array<char, 36> data_;
};

}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_UUID_H_
