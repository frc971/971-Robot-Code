#include "aos/util/scoped_pipe.h"

#include <fcntl.h>

#include <array>
#include <string>

#include "gtest/gtest.h"

namespace aos {
namespace util {
namespace testing {

// Tests using uint32_t read/write methods on the ScopedPipe objects.
TEST(ScopedPipeTest, IntegerPipe) {
  ScopedPipe::PipePair pipe = ScopedPipe::MakePipe();
  ASSERT_FALSE(pipe.read->Read().has_value())
      << "Shouldn't get anything on empty read.";
  pipe.write->Write(971);
  ASSERT_EQ(971, pipe.read->Read().value());
}

// Tests using string read/write methods on the ScopedPipe objects.
TEST(ScopedPipeTest, StringPipe) {
  ScopedPipe::PipePair pipe = ScopedPipe::MakePipe();
  std::string buffer;
  ASSERT_EQ(0u, pipe.read->Read(&buffer))
      << "Shouldn't get anything on empty read.";
  ASSERT_TRUE(buffer.empty());

  const char *const kAbc = "abcdef";
  pipe.write->Write(
      absl::Span<const uint8_t>(reinterpret_cast<const uint8_t *>(kAbc), 6));
  ASSERT_EQ(6u, pipe.read->Read(&buffer));
  ASSERT_EQ("abcdef", buffer);

  std::array<uint8_t, 10000> large_buffer;
  large_buffer.fill(99);
  pipe.write->Write(
      absl::Span<const uint8_t>(large_buffer.data(), large_buffer.size()));
  ASSERT_EQ(large_buffer.size(), pipe.read->Read(&buffer));
  for (size_t ii = 0; ii < large_buffer.size(); ++ii) {
    ASSERT_EQ(large_buffer[ii], buffer[ii + 6]);
  }
}

// Tests using string read/write methods on the ScopedPipe objects.
TEST(ScopedPipeTest, StringPipe2048) {
  ScopedPipe::PipePair pipe = ScopedPipe::MakePipe();
  std::string buffer;
  ASSERT_EQ(0u, pipe.read->Read(&buffer))
      << "Shouldn't get anything on empty read.";
  ASSERT_TRUE(buffer.empty());

  std::string a(2048, 'a');
  pipe.write->Write(absl::Span<const uint8_t>(
      reinterpret_cast<const uint8_t *>(a.data()), a.size()));
  ASSERT_EQ(2048u, pipe.read->Read(&buffer));
  ASSERT_EQ(a, buffer);
}

// Tests that calling SetCloexec succeeds and does indeed set FD_CLOEXEC.
TEST(ScopedPipeTest, SetCloexec) {
  ScopedPipe::PipePair pipe = ScopedPipe::MakePipe();
  ASSERT_EQ(0, fcntl(pipe.read->fd(), F_GETFD) & FD_CLOEXEC);
  pipe.read->SetCloexec();
  ASSERT_NE(0, fcntl(pipe.read->fd(), F_GETFD) & FD_CLOEXEC);
}

}  // namespace testing
}  // namespace util
}  // namespace aos
