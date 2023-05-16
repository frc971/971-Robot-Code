#include "aos/util/config_validator_lib.h"

#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "gtest/gtest-spi.h"

using aos::testing::ArtifactPath;
namespace aos::util::testing {

// Check that a reasonably normal config passes the config validator with a
// reasonable set of checks turned on.
TEST(ConfigValidatorTest, NoErrorOnValidConfigs) {
  const FlatbufferDetachedBuffer<Configuration> config =
      configuration::ReadConfig(
          ArtifactPath("aos/util/test_data/valid_multinode_config.json"));
  const FlatbufferDetachedBuffer<ConfigValidatorConfig> validator_config =
      JsonToFlatbuffer<ConfigValidatorConfig>(R"json({"logging": {
      "all_channels_logged": true,
      "logger_sets": [
        {
          "loggers": [],
          "replay_nodes": []
        },
        {
          "loggers": ["pi1"],
          "replay_nodes": ["pi1"]
        },
        {
          "loggers": ["pi2"],
          "replay_nodes": ["pi2"]
        }
      ]}})json");
  ConfigIsValid(&config.message(), &validator_config.message());
}

// Check that a reasonably normal single-node config passes the config validator
// with a reasonable set of checks turned on.
TEST(ConfigValidatorTest, NoErrorOnValidSingleNodeConfig) {
  const FlatbufferDetachedBuffer<Configuration> config =
      configuration::ReadConfig(
          ArtifactPath("aos/util/test_data/valid_singlenode_config.json"));
  const FlatbufferDetachedBuffer<ConfigValidatorConfig> validator_config =
      JsonToFlatbuffer<ConfigValidatorConfig>(R"json({"logging": {
      "all_channels_logged": true,
      "logger_sets": [
        {
          "loggers": [],
          "replay_nodes": []
        }
      ]}})json");
  ConfigIsValid(&config.message(), &validator_config.message());
}

// Checks that the validator fails if the message bridge statistics channels are
// missing.
TEST(ConfigValidatorTest, FailOnMissingStatisticsChannels) {
  EXPECT_FATAL_FAILURE(
      {
        const FlatbufferDetachedBuffer<Configuration> config =
            configuration::ReadConfig(ArtifactPath(
                "aos/util/test_data/multinode_no_statistics.json"));
        const FlatbufferDetachedBuffer<ConfigValidatorConfig> validator_config =
            JsonToFlatbuffer<ConfigValidatorConfig>("{}");
        ConfigIsValid(&config.message(), &validator_config.message());
      },
      "Statistics");
}

// Checks that the validator fails if a timestamp channel has a typo and so
// doesn't exist.
TEST(ConfigValidatorTest, FailOnTimestampTypo) {
  EXPECT_DEATH(
      {
        const FlatbufferDetachedBuffer<Configuration> config =
            configuration::ReadConfig(ArtifactPath(
                "aos/util/test_data/multinode_timestamp_typo.json"));
        const FlatbufferDetachedBuffer<ConfigValidatorConfig> validator_config =
            JsonToFlatbuffer<ConfigValidatorConfig>("{}");
        ConfigIsValid(&config.message(), &validator_config.message());
      },
      "not found in config");
}

// Checks that the validator fails if there is a RemoteMessage channel that is
// *not* a timestamp channel (Since this is almost always a typo).
TEST(ConfigValidatorTest, FailOnExtraneousTimestampChannel) {
  EXPECT_FATAL_FAILURE(
      {
        const FlatbufferDetachedBuffer<Configuration> config =
            configuration::ReadConfig(ArtifactPath(
                "aos/util/test_data/multinode_extraneous_timestamp.json"));
        const FlatbufferDetachedBuffer<ConfigValidatorConfig> validator_config =
            JsonToFlatbuffer<ConfigValidatorConfig>("{}");
        ConfigIsValid(&config.message(), &validator_config.message());
      },
      "linting failed");
}

// Checks that the validator fails on timestamp logger nodes that won't really
// log the timestamps.
TEST(ConfigValidatorTest, FailOnInvalidRemoteTimestampLogger) {
  EXPECT_FATAL_FAILURE(
      {
        const FlatbufferDetachedBuffer<Configuration> config =
            configuration::ReadConfig(
                ArtifactPath("aos/util/test_data/"
                             "multinode_invalid_timestamp_logger_list.json"));
        const FlatbufferDetachedBuffer<ConfigValidatorConfig> validator_config =
            JsonToFlatbuffer<ConfigValidatorConfig>(R"json({"logging": {
      "all_channels_logged": true,
      "logger_sets": [
        {
          "loggers": [],
          "replay_nodes": []
        }
      ]}})json");
        ConfigIsValid(&config.message(), &validator_config.message());
      },
      "linting failed");
}

// Checks that if you attempt to log on pi2 but expect it to have data for pi1
// then the test fails (at least, for a config which does not forward all the
// channels between the nodes).
TEST(ConfigValidatorTest, FailOnNormalInsufficientLogging) {
  EXPECT_NONFATAL_FAILURE(
      {
        const FlatbufferDetachedBuffer<Configuration> config =
            configuration::ReadConfig(
                ArtifactPath("aos/util/test_data/valid_multinode_config.json"));
        const FlatbufferDetachedBuffer<ConfigValidatorConfig> validator_config =
            JsonToFlatbuffer<ConfigValidatorConfig>(R"json({"logging": {
      "all_channels_logged": true,
      "logger_sets": [
        {
          "loggers": ["pi2"],
          "replay_nodes": ["pi1"]
        }
      ]}})json");
        ConfigIsValid(&config.message(), &validator_config.message());
      },
      "Failed to log");
}

// Checks that if we have a node that is configured to log all the data from all
// the nodes that the test passes.
TEST(ConfigValidatorTest, PassCommonLoggerNode) {
  const FlatbufferDetachedBuffer<Configuration> config =
      configuration::ReadConfig(
          ArtifactPath("aos/util/test_data/multinode_common_logger.json"));
  const FlatbufferDetachedBuffer<ConfigValidatorConfig> validator_config =
      JsonToFlatbuffer<ConfigValidatorConfig>(R"json({"logging": {
      "all_channels_logged": true,
      "logger_sets": [
        {
          "loggers": ["pi2"],
          "replay_nodes": ["pi1"]
        },
        {
          "loggers": [],
          "replay_nodes": []
        }
      ]}})json");
  ConfigIsValid(&config.message(), &validator_config.message());
}

// Sets up a config that will not actually log sufficient timestamp data to
// support full replay, and ensures that we identify that.
TEST(ConfigValidatorTest, FailOnInsufficientConfiguredTimestampData) {
  EXPECT_NONFATAL_FAILURE(
      {
        const FlatbufferDetachedBuffer<Configuration> config =
            configuration::ReadConfig(ArtifactPath(
                "aos/util/test_data/multinode_no_logged_timestamps.json"));
        const FlatbufferDetachedBuffer<ConfigValidatorConfig> validator_config =
            JsonToFlatbuffer<ConfigValidatorConfig>(R"json({"logging": {
      "all_channels_logged": true,
      "logger_sets": [
        {
          "loggers": [],
          "replay_nodes": []
        }
      ]}})json");
        ConfigIsValid(&config.message(), &validator_config.message());
      },
      R"json(Failed to log or replay any data on { "name": "/test", "type": "aos.examples.Ping" } from remote node pi2)json");
}

}  // namespace aos::util::testing
