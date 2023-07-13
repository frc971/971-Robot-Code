#include "aos/uuid.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <array>
#include <random>
#include <string_view>

#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(boot_uuid, "",
              "If set, override the boot UUID to have this value instead.");

namespace aos {
namespace {
void ToHex(const uint8_t *val, char *result, size_t count) {
  while (count > 0) {
    int upper = ((*val) >> 4) & 0xf;
    if (upper < 10) {
      result[0] = upper + '0';
    } else {
      result[0] = upper - 10 + 'a';
    }

    int lower = (*val) & 0xf;
    if (lower < 10) {
      result[1] = lower + '0';
    } else {
      result[1] = lower - 10 + 'a';
    }

    ++val;
    result += 2;
    --count;
  }
}

void FromHex(const char *val, uint8_t *result, size_t count) {
  while (count > 0) {
    CHECK((val[0] >= '0' && val[0] <= '9') || (val[0] >= 'a' && val[0] <= 'f'))
        << ": Invalid hex '" << val[0] << "'";
    CHECK((val[1] >= '0' && val[1] <= '9') || (val[1] >= 'a' && val[1] <= 'f'))
        << ": Invalid hex '" << val[1] << "'";

    uint8_t converted = 0;
    if (val[0] < 'a') {
      converted |= static_cast<uint8_t>(val[0] - '0') << 4;
    } else {
      converted |= (static_cast<uint8_t>(val[0] - 'a') + 0xa) << 4;
    }
    if (val[1] < 'a') {
      converted |= static_cast<uint8_t>(val[1] - '0');
    } else {
      converted |= (static_cast<uint8_t>(val[1] - 'a') + 0xa);
    }
    *result = converted;

    val += 2;
    ++result;
    --count;
  }
}

}  // namespace

namespace internal {
std::mt19937 FullySeededRandomGenerator() {
  // Total bits that the mt19937 has internally that we could plausibly
  // initialize with.
  // The internal state ends up being ~1200 bytes, which is significantly more
  // than the 128 bits we want for UUIDs, but since we should only need to
  // generate this randomness once, it should be fine.
  // If the performance cost ends up causing issues, then we can revisit the
  // need to *fully* seed the twister.
  constexpr size_t kInternalEntropy =
      std::mt19937::state_size * sizeof(std::mt19937::result_type);
  // Number, rounded up, of random values required.
  constexpr size_t kSeedsRequired =
      ((kInternalEntropy - 1) / sizeof(std::random_device::result_type)) + 1;
  std::random_device random_device;
// Older LLVM libstdc++'s just return 0 for the random device entropy.
#if !defined(__clang__) || (__clang_major__ > 13)
  CHECK_EQ(sizeof(std::random_device::result_type) * 8, random_device.entropy())
      << ": Does your random_device actually support generating entropy?";
#endif
  std::array<std::random_device::result_type, kSeedsRequired> random_data;
  std::generate(std::begin(random_data), std::end(random_data),
                std::ref(random_device));
  std::seed_seq seeds(std::begin(random_data), std::end(random_data));
  return std::mt19937(seeds);
}
}  // namespace internal

UUID UUID::Random() {
  // Note: This only provides 32 bits of randomness to each thread. However, by
  // keeping persistent pseudo-random number generators, we can increase the
  // overall randomness of each call to Random() (since, essentially, the
  // randomness is coming from the combination of the initial seed + the number
  // of times that Random() has been called in the given thread).
  //
  // thread_local to guarantee safe use of the generator itself.
  thread_local std::mt19937 gen(internal::FullySeededRandomGenerator());

  std::uniform_int_distribution<> dis(0, 255);
  UUID result;
  for (size_t i = 0; i < kDataSize; ++i) {
    result.data_[i] = dis(gen);
  }

  // Mark the reserved bits in the data that this is a uuid4, a random UUID.
  result.data_[6] = (result.data_[6] & 0x0f) | 0x40;
  result.data_[8] = (result.data_[6] & 0x3f) | 0x80;

  return result;
}

std::string UUID::ToString() const {
  std::string out;
  out.resize(UUID::kStringSize);
  CopyTo(out.data());
  return out;
}

std::ostream &operator<<(std::ostream &os, const UUID &uuid) {
  return os << uuid.ToString();
}

flatbuffers::Offset<flatbuffers::String> UUID::PackString(
    flatbuffers::FlatBufferBuilder *fbb) const {
  std::array<char, kStringSize> data;
  CopyTo(data.data());

  return fbb->CreateString(data.data(), data.size());
}

flatbuffers::Offset<flatbuffers::Vector<uint8_t>> UUID::PackVector(
    flatbuffers::FlatBufferBuilder *fbb) const {
  return fbb->CreateVector(data_.data(), data_.size());
}

void UUID::CopyTo(char *result) const {
  ToHex(&data_[0], result, 4);
  result[8] = '-';
  ToHex(&data_[4], result + 9, 2);
  result[13] = '-';
  ToHex(&data_[6], result + 14, 2);
  result[18] = '-';
  ToHex(&data_[8], result + 19, 2);
  result[23] = '-';
  ToHex(&data_[10], result + 24, 6);
}

UUID UUID::FromString(const flatbuffers::String *str) {
  return FromString(str->string_view());
}

UUID UUID::FromVector(const flatbuffers::Vector<uint8_t> *data) {
  CHECK(data != nullptr);
  CHECK_EQ(data->size(), kDataSize);

  UUID result;
  std::memcpy(result.data_.data(), data->Data(), kDataSize);
  return result;
}

UUID UUID::FromSpan(absl::Span<const uint8_t> data) {
  CHECK_EQ(data.size(), kDataSize);

  UUID result;
  std::copy(data.begin(), data.end(), result.data_.begin());
  return result;
}

UUID UUID::FromString(std::string_view str) {
  CHECK_EQ(str.size(), kStringSize);

  UUID result;
  FromHex(str.data(), result.data_.data(), 4);
  CHECK(str.data()[8] == '-' && str.data()[13] == '-' &&
        str.data()[18] == '-' && str.data()[23] == '-')
      << ": Invalid uuid.";
  FromHex(str.data() + 9, result.data_.data() + 4, 2);
  FromHex(str.data() + 14, result.data_.data() + 6, 2);
  FromHex(str.data() + 19, result.data_.data() + 8, 2);
  FromHex(str.data() + 24, result.data_.data() + 10, 6);
  return result;
}

UUID UUID::BootUUID() {
  if (!FLAGS_boot_uuid.empty()) {
    return UUID::FromString(FLAGS_boot_uuid);
  }

  int fd = open("/proc/sys/kernel/random/boot_id", O_RDONLY);
  PCHECK(fd != -1);

  std::array<char, kStringSize> data;
  CHECK_EQ(static_cast<ssize_t>(kStringSize),
           read(fd, data.begin(), kStringSize));
  close(fd);

  return UUID::FromString(std::string_view(data.data(), data.size()));
}

}  // namespace aos
