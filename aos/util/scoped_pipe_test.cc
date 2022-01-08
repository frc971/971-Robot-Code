#include "aos/util/scoped_pipe.h"

#include <array>
#include <string>

#include "gtest/gtest.h"

namespace aos {
namespace util {
namespace testing {

// Tests using uint32_t read/write methods on the ScopedPipe objects.
TEST(ScopedPipeTest, IntegerPipe) {
  std::tuple<ScopedPipe::ScopedReadPipe, ScopedPipe::ScopedWritePipe>
      pipe = ScopedPipe::MakePipe();
  ASSERT_FALSE(std::get<0>(pipe).Read().has_value())
      << "Shouldn't get anything on empty read.";
  std::get<1>(pipe).Write(971);
  ASSERT_EQ(971, std::get<0>(pipe).Read().value());
}

// Tests using string read/write methods on the ScopedPipe objects.
TEST(ScopedPipeTest, StringPipe) {
  std::tuple<ScopedPipe::ScopedReadPipe, ScopedPipe::ScopedWritePipe>
      pipe = ScopedPipe::MakePipe();
  std::string buffer;
  ASSERT_EQ(0u, std::get<0>(pipe).Read(&buffer))
      << "Shouldn't get anything on empty read.";
  ASSERT_TRUE(buffer.empty());

  const char *const kAbc = "abcdef";
  std::get<1>(pipe).Write(
      absl::Span<const uint8_t>(reinterpret_cast<const uint8_t *>(kAbc), 6));
  ASSERT_EQ(6u, std::get<0>(pipe).Read(&buffer));
  ASSERT_EQ("abcdef", buffer);

  std::array<uint8_t, 10000> large_buffer;
  large_buffer.fill(99);
  std::get<1>(pipe).Write(
      absl::Span<const uint8_t>(large_buffer.data(), large_buffer.size()));
  ASSERT_EQ(large_buffer.size(), std::get<0>(pipe).Read(&buffer));
  for (size_t ii = 0; ii < large_buffer.size(); ++ii) {
    ASSERT_EQ(large_buffer[ii], buffer[ii + 6]);
  }
}

}  // namespace testing
}  // namespace util
}  // namespace aos
