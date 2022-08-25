#ifndef AOS_EVENTS_LOGGING_UUID_H_
#define AOS_EVENTS_LOGGING_UUID_H_

#include <array>
#include <ostream>
#include <string>

#include "absl/types/span.h"
#include "aos/thread_local.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {

// Class to generate and hold a UUID.
class UUID {
 public:
  // Size of a UUID both as a string and the raw data.
  static constexpr size_t kStringSize = 36;
  static constexpr size_t kDataSize = 16;

  // Returns a randomly generated UUID.  This is known as a UUID4.
  static UUID Random();

  // Returns a uuid with all '0's.
  static constexpr UUID Zero() {
    UUID result;
    std::memset(result.data_.data(), 0, result.data_.size());
    return result;
  }

  // Converts a string UUID of the form 00000000-0000-0000-0000-000000000000 to
  // a UUID.
  static UUID FromString(std::string_view string);
  static UUID FromString(const flatbuffers::String *string);

  // Converts a 16 byte vector (128 bits) to a UUID.  This requires no
  // transformation.
  static UUID FromVector(const flatbuffers::Vector<uint8_t> *data);

  // Returns the boot UUID for the current linux computer.
  static UUID BootUUID();

  // Default constructor which builds an uninitialized UUID.  Use one of the
  // static methods if you want something more useful.
  constexpr UUID() : data_() {}
  constexpr UUID(const UUID &uuid) = default;

  constexpr UUID &operator=(const UUID &other) = default;

  // Packs this UUID into a flatbuffer as a string.
  flatbuffers::Offset<flatbuffers::String> PackString(
      flatbuffers::FlatBufferBuilder *fbb) const;
  // Copies this UUID as a string into the memory pointed by result.  Result
  // must be at least kStringSize long.
  void CopyTo(char *result) const;
  // Returns this UUID as a string.
  std::string ToString() const;

  // Packs the UUID bytes directly into a vector.
  flatbuffers::Offset<flatbuffers::Vector<uint8_t>> PackVector(
      flatbuffers::FlatBufferBuilder *fbb) const;

  // Returns a human-readable string representing this UUID.
  //
  // This is done without any memory allocation, which means it's returned in a
  // thread-local buffer.
  //
  // Be careful using this. It's mostly useful for low-level tracing of UUIDs
  // through the system.
  const char *thread_local_string() const {
    AOS_THREAD_LOCAL char buffer[kStringSize + 1];
    CopyTo(buffer);
    return buffer;
  }

  // Returns the underlying UUID data.
  absl::Span<const uint8_t> span() const {
    return absl::Span<const uint8_t>(data_.data(), data_.size());
  }

  bool operator==(const UUID &other) const { return other.span() == span(); }
  bool operator<(const UUID &other) const { return other.span() < span(); }
  bool operator!=(const UUID &other) const { return other.span() != span(); }

 private:
  friend std::ostream &operator<<(std::ostream &os, const UUID &uuid);

  // Encoded storage for the data.
  std::array<uint8_t, kDataSize> data_;
};

std::ostream &operator<<(std::ostream &os, const UUID &uuid);

}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_UUID_H_
