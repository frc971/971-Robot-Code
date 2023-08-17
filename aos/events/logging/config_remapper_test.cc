#include "aos/events/logging/config_remapper.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/events/event_loop_generated.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/multinode_logger_test_lib.h"
#include "aos/events/message_counter.h"
#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/timestamp_generated.h"
#include "aos/testing/tmpdir.h"
#include "multinode_logger_test_lib.h"

namespace aos {
namespace testing {
using namespace logger::testing;
using namespace logger;
namespace chrono = std::chrono;

using ConfigRemapperTest = MultinodeLoggerTest;

INSTANTIATE_TEST_SUITE_P(
    All, ConfigRemapperTest,
    ::testing::Combine(
        ::testing::Values(
            ConfigParams{"multinode_pingpong_combined_config.json", true,
                         kCombinedConfigSha1(), kCombinedConfigSha1(),
                         FileStrategy::kCombine,
                         ForceTimestampBuffering::kForceBufferTimestamps},
            ConfigParams{"multinode_pingpong_split_config.json", false,
                         kSplitConfigSha1(), kReloggedSplitConfigSha1(),
                         FileStrategy::kCombine,
                         ForceTimestampBuffering::kForceBufferTimestamps}),
        ::testing::ValuesIn(SupportedCompressionAlgorithms())));

// Tests that we can read a config and remap a channel
TEST_P(ConfigRemapperTest, RemapOriginalChannel) {
  ConfigRemapper remapper(&config_.message());

  remapper.RemapOriginalChannel<examples::Ping>("/test");

  const Channel *channel = configuration::GetChannel<examples::Ping>(
      remapper.remapped_configuration(), "/original/test", "pi1", nullptr);
  EXPECT_NE(channel, nullptr);
  EXPECT_EQ(channel->name()->string_view(), "/original/test");
  EXPECT_EQ(channel->type()->string_view(), "aos.examples.Ping");
}

// Tests that we can read a config and rename a channel
TEST_P(ConfigRemapperTest, RenameOriginalChannel) {
  ConfigRemapper remapper(&config_.message());

  remapper.RenameOriginalChannel<examples::Ping>("/test", "/original/test");

  const Channel *channel = configuration::GetChannel<examples::Ping>(
      remapper.remapped_configuration(), "/original/test", "pi1", nullptr);
  EXPECT_NE(channel, nullptr);
  EXPECT_EQ(channel->name()->string_view(), "/original/test");
  EXPECT_EQ(channel->type()->string_view(), "aos.examples.Ping");
}

// Tests that we can remap a channel specifying a certain node
TEST_P(ConfigRemapperTest, RemapOriginalChannelWithNode) {
  ConfigRemapper remapper(&config_.message());

  const Node *node =
      configuration::GetNode(remapper.remapped_configuration(), "pi1");

  // Remap just on pi1.
  remapper.RemapOriginalChannel<aos::timing::Report>("/aos", node);

  const Channel *channel = configuration::GetChannel<aos::timing::Report>(
      remapper.remapped_configuration(), "/original/pi1/aos", "pi1", node);
  EXPECT_NE(channel, nullptr);
  EXPECT_EQ(channel->name()->string_view(), "/original/pi1/aos");
  EXPECT_EQ(channel->type()->string_view(), "aos.timing.Report");
}

// Tests that we can rename a channel specifying a certain node
TEST_P(ConfigRemapperTest, RenameOriginalChannelWithNode) {
  ConfigRemapper remapper(&config_.message());

  const Node *node =
      configuration::GetNode(remapper.remapped_configuration(), "pi1");

  // Rename just on pi1.
  remapper.RenameOriginalChannel<aos::timing::Report>("/aos", node,
                                                      "/original/pi1/aos");

  const Channel *channel = configuration::GetChannel<aos::timing::Report>(
      remapper.remapped_configuration(), "/original/pi1/aos", "pi1", node);
  EXPECT_NE(channel, nullptr);
  EXPECT_EQ(channel->name()->string_view(), "/original/pi1/aos");
  EXPECT_EQ(channel->type()->string_view(), "aos.timing.Report");
}

}  // namespace testing
}  // namespace aos
