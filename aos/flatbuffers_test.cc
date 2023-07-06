#include "aos/flatbuffers.h"

#include "absl/strings/str_cat.h"
#include "gtest/gtest.h"

#include "aos/json_to_flatbuffer.h"
#include "aos/json_to_flatbuffer_generated.h"
#include "aos/testing/tmpdir.h"

namespace aos {
namespace testing {

// Tests that Verify works.
TEST(FlatbufferTest, Verify) {
  FlatbufferDetachedBuffer<Configuration> fb =
      JsonToFlatbuffer<Configuration>("{}");
  FlatbufferSpan<Configuration> fb_span(fb);
  EXPECT_TRUE(fb.Verify());
  EXPECT_TRUE(fb_span.Verify());

  // Now confirm it works on an empty flatbuffer.
  FlatbufferSpan<Configuration> empty(absl::Span<const uint8_t>(nullptr, 0));
  EXPECT_FALSE(empty.Verify());
}

// Tests the ability to map a flatbuffer on disk to memory
TEST(FlatbufferMMapTest, Verify) {
  FlatbufferDetachedBuffer<Configuration> fb =
      JsonToFlatbuffer<Configuration>("{\"foo_int\": 3}");

  const std::string fb_path = absl::StrCat(TestTmpDir(), "/fb.bfbs");
  WriteFlatbufferToFile(fb_path, fb);

  FlatbufferMMap<Configuration> fb_mmap(fb_path);
  EXPECT_TRUE(fb.Verify());
  EXPECT_TRUE(fb_mmap.Verify());
  ASSERT_EQ(fb_mmap.message().foo_int(), 3);

  // Verify that copying works
  {
    FlatbufferMMap<Configuration> fb_mmap2(fb_path);
    fb_mmap2 = fb_mmap;
    EXPECT_TRUE(fb_mmap.Verify());
    EXPECT_TRUE(fb_mmap2.Verify());
    ASSERT_EQ(fb_mmap2.message().foo_int(), 3);
    ASSERT_EQ(fb_mmap.message().foo_int(), 3);
  }
  EXPECT_TRUE(fb_mmap.Verify());
  ASSERT_EQ(fb_mmap.message().foo_int(), 3);

  // Verify that moving works
  {
    FlatbufferMMap<Configuration> fb_mmap3(fb_path);
    fb_mmap3 = std::move(fb_mmap);
    EXPECT_TRUE(fb_mmap3.Verify());
    ASSERT_EQ(fb_mmap3.message().foo_int(), 3);
  }
}

// Tests the ability to modify a flatbuffer mmaped from on disk in memory
TEST(FlatbufferMMapTest, Writeable) {
  FlatbufferDetachedBuffer<Configuration> fb =
      JsonToFlatbuffer<Configuration>("{\"foo_int\": 3}");

  const std::string fb_path = absl::StrCat(TestTmpDir(), "/fb.bfbs");
  WriteFlatbufferToFile(fb_path, fb);

  {
    FlatbufferMMap<Configuration> fb_mmap(fb_path,
                                          util::FileOptions::kWriteable);
    fb_mmap.mutable_message()->mutate_foo_int(5);
  }

  {
    FlatbufferMMap<Configuration> fb_mmap(fb_path);
    EXPECT_EQ(fb_mmap.message().foo_int(), 5);
  }
}

}  // namespace testing
}  // namespace aos
