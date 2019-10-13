#include "aos/configuration.h"

#include "absl/strings/strip.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/test_logging.h"
#include "aos/util/file.h"
#include "gtest/gtest.h"

namespace aos {
namespace configuration {
namespace testing {

class ConfigurationTest : public ::testing::Test {
 public:
  ConfigurationTest() { ::aos::testing::EnableTestLogging(); }
};

typedef ConfigurationTest ConfigurationDeathTest;

// *the* expected location for all working tests.
const char *kExpectedLocation =
    "{ \"name\": \"/foo\", \"type\": \".aos.bar\", \"max_size\": 5 }";

// Tests that we can read and merge a configuration.
TEST_F(ConfigurationTest, ConfigMerge) {
  Flatbuffer<Configuration> config = ReadConfig("aos/testdata/config1.json");
  printf("Read: %s\n", FlatbufferToJson(config, true).c_str());

  EXPECT_EQ(
      absl::StripSuffix(
          util::ReadFileToStringOrDie("aos/testdata/expected.json"), "\n"),
      FlatbufferToJson(config, true));
}

// Tests that we die when a file is imported twice.
TEST_F(ConfigurationDeathTest, DuplicateFile) {
  EXPECT_DEATH(
      {
        Flatbuffer<Configuration> config =
            ReadConfig("aos/testdata/config1_bad.json");
      },
      "aos/testdata/config1_bad.json");
}

// Tests that we can lookup a location, complete with maps, from a merged
// config.
TEST_F(ConfigurationTest, GetLocation) {
  Flatbuffer<Configuration> config = ReadConfig("aos/testdata/config1.json");

  // Test a basic lookup first.
  EXPECT_EQ(FlatbufferToJson(GetLocation(config, "/foo", ".aos.bar", "app1")),
            kExpectedLocation);

  // Test that an invalid name results in nullptr back.
  EXPECT_EQ(GetLocation(config, "/invalid_name", ".aos.bar", "app1"), nullptr);

  // Tests that a root map/rename works. And that they get processed from the
  // bottom up.
  EXPECT_EQ(
      FlatbufferToJson(GetLocation(config, "/batman", ".aos.bar", "app1")),
      kExpectedLocation);

  // And then test that an application specific map/rename works.
  EXPECT_EQ(FlatbufferToJson(GetLocation(config, "/bar", ".aos.bar", "app1")),
            kExpectedLocation);
  EXPECT_EQ(FlatbufferToJson(GetLocation(config, "/baz", ".aos.bar", "app2")),
            kExpectedLocation);

  // And then test that an invalid application name gets properly ignored.
  EXPECT_EQ(FlatbufferToJson(GetLocation(config, "/foo", ".aos.bar", "app3")),
            kExpectedLocation);
}

}  // namespace testing
}  // namespace configuration
}  // namespace aos
