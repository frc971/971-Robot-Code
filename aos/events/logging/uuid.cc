#include "aos/events/logging/uuid.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <array>
#include <random>
#include <string_view>

#include "glog/logging.h"

namespace aos {
namespace {
char ToHex(int val) {
  if (val < 10) {
    return val + '0';
  } else {
    return val - 10 + 'a';
  }
}
}  // namespace

UUID UUID::Random() {
  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_int_distribution<> dis(0, 15);
  std::uniform_int_distribution<> dis2(8, 11);

  UUID result;

  // UUID4 is implemented per https://www.cryptosys.net/pki/uuid-rfc4122.html
  int i;
  for (i = 0; i < 8; i++) {
    result.data_[i] = ToHex(dis(gen));
  }
  result.data_[i] = '-';
  ++i;
  for (; i < 13; i++) {
    result.data_[i] = ToHex(dis(gen));
  }
  result.data_[i] = '-';
  ++i;
  result.data_[i] = '4';
  ++i;
  for (; i < 18; i++) {
    result.data_[i] = ToHex(dis(gen));
  }
  result.data_[i] = '-';
  ++i;
  result.data_[i] = ToHex(dis2(gen));
  ++i;
  for (; i < 23; i++) {
    result.data_[i] = ToHex(dis(gen));
  }
  result.data_[i] = '-';
  ++i;
  for (; i < 36; i++) {
    result.data_[i] = ToHex(dis(gen));
  }

  return result;
}

UUID UUID::Zero() { return FromString("00000000-0000-0000-0000-000000000000"); }

UUID UUID::FromString(std::string_view str) {
  UUID result;
  CHECK_EQ(str.size(), kSize);

  std::copy(str.begin(), str.end(), result.data_.begin());
  return result;
}

UUID UUID::BootUUID() {
  int fd = open("/proc/sys/kernel/random/boot_id", O_RDONLY);
  PCHECK(fd != -1);

  UUID result;
  CHECK_EQ(static_cast<ssize_t>(kSize), read(fd, result.data_.begin(), kSize));
  close(fd);

  return result;
}

}  // namespace aos
