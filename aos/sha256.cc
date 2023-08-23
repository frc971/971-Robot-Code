#include "aos/sha256.h"

#include <iomanip>
#include <sstream>
#include <string>

#include "absl/types/span.h"
#include "openssl/sha.h"

#include "aos/util/file.h"

namespace aos {

std::string Sha256(const absl::Span<const uint8_t> str) {
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256_CTX sha256;
  SHA256_Init(&sha256);
  SHA256_Update(&sha256, str.data(), str.size());
  SHA256_Final(hash, &sha256);
  std::stringstream ss;
  for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
    ss << std::hex << std::setw(2) << std::setfill('0')
       << static_cast<int>(hash[i]);
  }
  return ss.str();
}

std::string Sha256(std::string_view str) {
  return Sha256({reinterpret_cast<const uint8_t *>(str.data()), str.size()});
}

std::string Sha256OfFile(std::filesystem::path file) {
  const std::string contents = aos::util::ReadFileToStringOrDie(file.string());
  return Sha256(contents);
}

}  // namespace aos
