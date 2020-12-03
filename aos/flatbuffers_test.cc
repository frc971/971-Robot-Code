#include "aos/flatbuffers.h"

#include "gtest/gtest.h"

#include "aos/json_to_flatbuffer.h"
#include "aos/json_to_flatbuffer_generated.h"

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

}  // namespace testing
}  // namespace aos
