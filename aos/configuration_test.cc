#include "aos/configuration.h"

#include "absl/strings/strip.h"
#include "aos/events/ping_generated.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/flatbuffer_eq.h"
#include "aos/testing/path.h"
#include "aos/testing/test_logging.h"
#include "aos/util/file.h"
#include "flatbuffers/reflection.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace aos {
namespace configuration {
namespace testing {

using aos::testing::ArtifactPath;
namespace chrono = std::chrono;

class ConfigurationTest : public ::testing::Test {
 public:
  ConfigurationTest() { ::aos::testing::EnableTestLogging(); }
};

typedef ConfigurationTest ConfigurationDeathTest;

// *the* expected location for all working tests.
aos::FlatbufferDetachedBuffer<Channel> ExpectedLocation() {
  return JsonToFlatbuffer<Channel>(
      "{ \"name\": \"/foo\", \"type\": \".aos.bar\", \"max_size\": 5 }");
}

// And for multinode setups
aos::FlatbufferDetachedBuffer<Channel> ExpectedMultinodeLocation() {
  return JsonToFlatbuffer<Channel>(
      "{ \"name\": \"/foo\", \"type\": \".aos.bar\", \"max_size\": 5, "
      "\"source_node\": \"pi1\" }");
}

// Tests that we can read and merge a configuration.
TEST_F(ConfigurationTest, ConfigMerge) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));
  LOG(INFO) << "Read: " << FlatbufferToJson(config, {.multi_line = true});

  EXPECT_EQ(absl::StripSuffix(util::ReadFileToStringOrDie(
                                  ArtifactPath("aos/testdata/expected.json")),
                              "\n"),
            FlatbufferToJson(config, {.multi_line = true}));
}

// Tests that we can get back a ChannelIndex.
TEST_F(ConfigurationTest, ChannelIndex) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));

  EXPECT_EQ(
      ChannelIndex(&config.message(), config.message().channels()->Get(1u)),
      1u);
}

// Tests that we can read and merge a multinode configuration.
TEST_F(ConfigurationTest, ConfigMergeMultinode) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1_multinode.json"));
  LOG(INFO) << "Read: " << FlatbufferToJson(config, {.multi_line = true});

  EXPECT_EQ(std::string(absl::StripSuffix(
                util::ReadFileToStringOrDie(
                    ArtifactPath("aos/testdata/expected_multinode.json")),
                "\n")),
            FlatbufferToJson(config, {.multi_line = true}));
}

// Tests that we sort the entries in a config so we can look entries up.
TEST_F(ConfigurationTest, UnsortedConfig) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/backwards.json"));

  LOG(INFO) << "Read: " << FlatbufferToJson(config, {.multi_line = true});

  EXPECT_EQ(FlatbufferToJson(GetChannel(config, "/aos/robot_state",
                                        "aos.RobotState", "app1", nullptr)),
            "{ \"name\": \"/aos/robot_state\", \"type\": \"aos.RobotState\", "
            "\"max_size\": 5 }");
}

// Tests that we die when a file is imported twice.
TEST_F(ConfigurationDeathTest, DuplicateFile) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/config1_bad.json"));
      },
      "aos/testdata/config1_bad.json");
}

// Tests that we die when we give an invalid path.
TEST_F(ConfigurationDeathTest, NonexistentFile) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig("nonexistent/config.json");
      },
      "above error");
}

// Tests that we return std::nullopt when we give an invalid path.
TEST_F(ConfigurationTest, NonexistentFileOptional) {
  std::optional<FlatbufferDetachedBuffer<Configuration>> config =
      MaybeReadConfig("nonexistent/config.json");
  EXPECT_FALSE(config.has_value());
}

// Tests that we reject invalid channel names.  This means any channels with //
// in their name, a trailing /, or regex characters.
TEST_F(ConfigurationDeathTest, InvalidChannelName) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/invalid_channel_name1.json"));
      },
      "Channel names can't end with '/'");
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/invalid_channel_name2.json"));
      },
      "Invalid channel name");
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/invalid_channel_name3.json"));
        LOG(FATAL) << "Foo";
      },
      "Invalid channel name");
}

// Tests that we can modify a config with a json snippet.
TEST_F(ConfigurationTest, MergeWithConfig) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));
  LOG(INFO) << "Read: " << FlatbufferToJson(config, {.multi_line = true});

  FlatbufferDetachedBuffer<Configuration> updated_config =
      MergeWithConfig(&config.message(),
                      R"channel({
  "channels": [
    {
      "name": "/foo",
      "type": ".aos.bar",
      "max_size": 100
    }
  ]
})channel");

  EXPECT_EQ(absl::StripSuffix(util::ReadFileToStringOrDie(ArtifactPath(
                                  "aos/testdata/expected_merge_with.json")),
                              "\n"),
            FlatbufferToJson(updated_config, {.multi_line = true}));
}

// Tests that we can lookup a location, complete with maps, from a merged
// config.
TEST_F(ConfigurationTest, GetChannel) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));

  // Test a basic lookup first.
  EXPECT_THAT(GetChannel(config, "/foo", ".aos.bar", "app1", nullptr),
              aos::testing::FlatbufferEq(ExpectedLocation()));

  // Test that an invalid name results in nullptr back.
  EXPECT_EQ(GetChannel(config, "/invalid_name", ".aos.bar", "app1", nullptr),
            nullptr);

  // Tests that a root map/rename works. And that they get processed from the
  // bottom up.
  EXPECT_THAT(GetChannel(config, "/batman", ".aos.bar", "app1", nullptr),
              aos::testing::FlatbufferEq(ExpectedLocation()));

  // And then test that an application specific map/rename works.
  EXPECT_THAT(GetChannel(config, "/bar", ".aos.bar", "app1", nullptr),
              aos::testing::FlatbufferEq(ExpectedLocation()));
  EXPECT_THAT(GetChannel(config, "/baz", ".aos.bar", "app2", nullptr),
              aos::testing::FlatbufferEq(ExpectedLocation()));

  // And then test that an invalid application name gets properly ignored.
  EXPECT_THAT(GetChannel(config, "/foo", ".aos.bar", "app3", nullptr),
              aos::testing::FlatbufferEq(ExpectedLocation()));
}

// Tests that we can do reverse-lookups of channel names.
TEST_F(ConfigurationTest, GetChannelAliases) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));

  // Test a basic lookup first.
  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo", ".aos.bar", "app1", nullptr),
      ::testing::UnorderedElementsAre("/foo", "/batman", "/bar"));
  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/bar", ".aos.bar", "app1", nullptr),
      ::testing::UnorderedElementsAre("/batman", "/bar"));
  EXPECT_THAT(GetChannelAliases(&config.message(), "/batman", ".aos.bar",
                                "app1", nullptr),
              ::testing::UnorderedElementsAre("/batman"));
  // /bar (deliberately) does not get included because of the ordering in the
  // map.
  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo", ".aos.bar", "", nullptr),
      ::testing::UnorderedElementsAre("/foo", "/batman"));
  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo", ".aos.bar", "app2", nullptr),
      ::testing::UnorderedElementsAre("/foo", "/batman", "/baz"));
}

// Tests that we can lookup a location with node specific maps.
TEST_F(ConfigurationTest, GetChannelMultinode) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  const Node *pi1 = GetNode(&config.message(), "pi1");
  const Node *pi2 = GetNode(&config.message(), "pi2");

  // Test a basic lookup first.
  EXPECT_THAT(GetChannel(config, "/foo", ".aos.bar", "app1", pi1),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));
  EXPECT_THAT(GetChannel(config, "/foo", ".aos.bar", "app1", pi2),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));

  // Tests that a root map/rename works with a node specific map.
  EXPECT_THAT(GetChannel(config, "/batman", ".aos.bar", "app1", pi1),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));

  // Tests that a root map/rename fails with a node specific map for the wrong
  // node.
  EXPECT_EQ(GetChannel(config, "/batman", ".aos.bar", "app1", pi2), nullptr);

  // And then test that an application specific map/rename works.
  EXPECT_THAT(GetChannel(config, "/batman2", ".aos.bar", "app1", pi1),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));
  EXPECT_THAT(GetChannel(config, "/batman3", ".aos.bar", "app1", pi1),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));

  // And then that it fails when the node changes.
  EXPECT_EQ(GetChannel(config, "/batman3", ".aos.bar", "app1", pi2), nullptr);
}

// Tests that reverse channel lookup on a multi-node config (including with
// wildcards) works.
TEST_F(ConfigurationTest, GetChannelAliasesMultinode) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));

  const Node *pi1 = GetNode(&config.message(), "pi1");
  const Node *pi2 = GetNode(&config.message(), "pi2");

  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo", ".aos.bar", "app1", pi1),
      ::testing::UnorderedElementsAre("/foo", "/batman", "/batman2", "/batman3",
                                      "/magic/string"));

  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo", ".aos.baz", "app1", pi1),
      ::testing::UnorderedElementsAre("/foo", "/batman3", "/magic/string"));

  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo/testing", ".aos.bar", "", pi1),
      ::testing::UnorderedElementsAre("/foo/testing", "/magic/string/testing"));

  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo/testing", ".aos.bar", "app1",
                        pi2),
      ::testing::UnorderedElementsAre("/foo/testing", "/magic/string/testing"));
}

// Tests that we can lookup a location with type specific maps.
TEST_F(ConfigurationTest, GetChannelTypedMultinode) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  const Node *pi1 = GetNode(&config.message(), "pi1");

  // Test a basic lookup first.
  EXPECT_THAT(GetChannel(config, "/batman", ".aos.bar", "app1", pi1),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));

  // Now confirm that a second message on the same name doesn't get remapped.
  const char *kExpectedBazMultinodeLocation =
      "{ \"name\": \"/batman\", \"type\": \".aos.baz\", \"max_size\": 5, "
      "\"source_node\": \"pi1\" }";
  EXPECT_EQ(
      FlatbufferToJson(GetChannel(config, "/batman", ".aos.baz", "app1", pi1)),
      kExpectedBazMultinodeLocation);
}

// Tests that we can lookup a location with a glob
TEST_F(ConfigurationTest, GetChannelGlob) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  const Node *pi1 = GetNode(&config.message(), "pi1");

  // Confirm that a glob with nothing after it matches.
  EXPECT_THAT(GetChannel(config, "/magic/string", ".aos.bar", "app7", pi1),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));

  // Now confirm that glob with something following it matches and renames
  // correctly.
  const char *kExpectedSubfolderMultinodeLocation =
      "{ \"name\": \"/foo/subfolder\", \"type\": \".aos.bar\", \"max_size\": "
      "5, \"source_node\": \"pi1\" }";
  EXPECT_EQ(FlatbufferToJson(GetChannel(config, "/magic/string/subfolder",
                                        ".aos.bar", "app7", pi1)),
            kExpectedSubfolderMultinodeLocation);
}

// Tests that we reject a configuration which has a nodes list, but has channels
// withoout source_node filled out.
TEST_F(ConfigurationDeathTest, InvalidSourceNode) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/invalid_nodes.json"));
      },
      "source_node");

  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/invalid_source_node.json"));
      },
      "source_node");

  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config = ReadConfig(
            ArtifactPath("aos/testdata/invalid_destination_node.json"));
      },
      "destination_nodes");

  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/self_forward.json"));
      },
      "forwarding data to itself");
}

// Tests that our node writeable helpers work as intended.
TEST_F(ConfigurationTest, ChannelIsSendableOnNode) {
  FlatbufferDetachedBuffer<Channel> good_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "foo"
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> bad_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar"
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> node(JsonToFlatbuffer(
      R"node({
  "name": "foo"
})node",
      Node::MiniReflectTypeTable()));

  EXPECT_TRUE(
      ChannelIsSendableOnNode(&good_channel.message(), &node.message()));
  EXPECT_FALSE(
      ChannelIsSendableOnNode(&bad_channel.message(), &node.message()));
}

// Tests that our node readable and writeable helpers work as intended.
TEST_F(ConfigurationTest, ChannelIsReadableOnNode) {
  FlatbufferDetachedBuffer<Channel> good_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "destination_nodes": [
    {
      "name": "baz"
    },
    {
      "name": "foo"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> bad_channel1(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar"
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> bad_channel2(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> node(JsonToFlatbuffer(
      R"node({
  "name": "foo"
})node",
      Node::MiniReflectTypeTable()));

  EXPECT_TRUE(
      ChannelIsReadableOnNode(&good_channel.message(), &node.message()));
  EXPECT_FALSE(
      ChannelIsReadableOnNode(&bad_channel1.message(), &node.message()));
  EXPECT_FALSE(
      ChannelIsReadableOnNode(&bad_channel2.message(), &node.message()));
}

// Tests that our node message is logged helpers work as intended.
TEST_F(ConfigurationTest, ChannelMessageIsLoggedOnNode) {
  FlatbufferDetachedBuffer<Channel> logged_on_self_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> not_logged_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "NOT_LOGGED",
  "destination_nodes": [
    {
      "name": "baz",
      "timestamp_logger": "LOCAL_LOGGER"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_remote_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "REMOTE_LOGGER",
  "logger_nodes": ["baz"],
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_separate_logger_node_channel(
      JsonToFlatbuffer(
          R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "REMOTE_LOGGER",
  "logger_nodes": ["foo"],
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
          Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_both_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "LOCAL_AND_REMOTE_LOGGER",
  "logger_nodes": ["baz"],
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> foo_node(JsonToFlatbuffer(
      R"node({
  "name": "foo"
})node",
      Node::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> bar_node(JsonToFlatbuffer(
      R"node({
  "name": "bar"
})node",
      Node::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> baz_node(JsonToFlatbuffer(
      R"node({
  "name": "baz"
})node",
      Node::MiniReflectTypeTable()));

  // Local logger.
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&logged_on_self_channel.message(),
                                            &foo_node.message()));
  EXPECT_TRUE(ChannelMessageIsLoggedOnNode(&logged_on_self_channel.message(),
                                           &bar_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&logged_on_self_channel.message(),
                                            &baz_node.message()));
  EXPECT_TRUE(
      ChannelMessageIsLoggedOnNode(&logged_on_self_channel.message(), nullptr));

  // No logger.
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&not_logged_channel.message(),
                                            &foo_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&not_logged_channel.message(),
                                            &bar_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&not_logged_channel.message(),
                                            &baz_node.message()));
  EXPECT_FALSE(
      ChannelMessageIsLoggedOnNode(&not_logged_channel.message(), nullptr));

  // Remote logger.
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&logged_on_remote_channel.message(),
                                            &foo_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&logged_on_remote_channel.message(),
                                            &bar_node.message()));
  EXPECT_TRUE(ChannelMessageIsLoggedOnNode(&logged_on_remote_channel.message(),
                                           &baz_node.message()));

  // Separate logger.
  EXPECT_TRUE(ChannelMessageIsLoggedOnNode(
      &logged_on_separate_logger_node_channel.message(), &foo_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(
      &logged_on_separate_logger_node_channel.message(), &bar_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(
      &logged_on_separate_logger_node_channel.message(), &baz_node.message()));

  // Logged in multiple places.
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&logged_on_both_channel.message(),
                                            &foo_node.message()));
  EXPECT_TRUE(ChannelMessageIsLoggedOnNode(&logged_on_both_channel.message(),
                                           &bar_node.message()));
  EXPECT_TRUE(ChannelMessageIsLoggedOnNode(&logged_on_both_channel.message(),
                                           &baz_node.message()));
}

// Tests that our node message is logged helpers work as intended.
TEST_F(ConfigurationDeathTest, ChannelMessageIsLoggedOnNode) {
  FlatbufferDetachedBuffer<Channel> logged_on_both_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "LOCAL_AND_REMOTE_LOGGER",
  "logger_nodes": ["baz"],
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_separate_logger_node_channel(
      JsonToFlatbuffer(
          R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "REMOTE_LOGGER",
  "logger_nodes": ["foo"],
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
          Channel::MiniReflectTypeTable()));

  EXPECT_DEATH(
      {
        ChannelMessageIsLoggedOnNode(&logged_on_both_channel.message(),
                                     nullptr);
      },
      "Unsupported logging configuration in a single node world");
  EXPECT_DEATH(
      {
        ChannelMessageIsLoggedOnNode(
            &logged_on_separate_logger_node_channel.message(), nullptr);
      },
      "Unsupported logging configuration in a single node world");
}

// Tests that our forwarding timestamps are logged helpers work as intended.
TEST_F(ConfigurationTest, ConnectionDeliveryTimeIsLoggedOnNode) {
  FlatbufferDetachedBuffer<Channel> logged_on_self_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "REMOTE_LOGGER",
  "logger_nodes": ["baz"],
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> not_logged_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "NOT_LOGGED",
  "destination_nodes": [
    {
      "name": "baz",
      "timestamp_logger": "NOT_LOGGED"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_remote_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "destination_nodes": [
    {
      "name": "baz",
      "timestamp_logger": "REMOTE_LOGGER",
      "timestamp_logger_nodes": ["bar"]
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_separate_logger_node_channel(
      JsonToFlatbuffer(
          R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "REMOTE_LOGGER",
  "logger_nodes": ["foo"],
  "destination_nodes": [
    {
      "name": "baz",
      "timestamp_logger": "REMOTE_LOGGER",
      "timestamp_logger_nodes": ["foo"]
    }
  ]
})channel",
          Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_both_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "destination_nodes": [
    {
      "name": "baz",
      "timestamp_logger": "LOCAL_AND_REMOTE_LOGGER",
      "timestamp_logger_nodes": ["bar"]
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> foo_node(JsonToFlatbuffer(
      R"node({
  "name": "foo"
})node",
      Node::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> bar_node(JsonToFlatbuffer(
      R"node({
  "name": "bar"
})node",
      Node::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> baz_node(JsonToFlatbuffer(
      R"node({
  "name": "baz"
})node",
      Node::MiniReflectTypeTable()));

  // Local logger.
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_self_channel.message(), &baz_node.message(),
      &foo_node.message()));
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_self_channel.message(), &baz_node.message(),
      &bar_node.message()));
  EXPECT_TRUE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_self_channel.message(), &baz_node.message(),
      &baz_node.message()));

  // No logger means.
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &not_logged_channel.message(), &baz_node.message(), &foo_node.message()));
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &not_logged_channel.message(), &baz_node.message(), &bar_node.message()));
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &not_logged_channel.message(), &baz_node.message(), &baz_node.message()));

  // Remote logger.
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_remote_channel.message(), &baz_node.message(),
      &foo_node.message()));
  EXPECT_TRUE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_remote_channel.message(), &baz_node.message(),
      &bar_node.message()));
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_remote_channel.message(), &baz_node.message(),
      &baz_node.message()));

  // Separate logger.
  EXPECT_TRUE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_separate_logger_node_channel.message(), &baz_node.message(),
      &foo_node.message()));
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_separate_logger_node_channel.message(), &baz_node.message(),
      &bar_node.message()));
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_separate_logger_node_channel.message(), &baz_node.message(),
      &baz_node.message()));

  // Logged on both the node and a remote node.
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_both_channel.message(), &baz_node.message(),
      &foo_node.message()));
  EXPECT_TRUE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_both_channel.message(), &baz_node.message(),
      &bar_node.message()));
  EXPECT_TRUE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_both_channel.message(), &baz_node.message(),
      &baz_node.message()));
}

// Tests that we can deduce source nodes from a multinode config.
TEST_F(ConfigurationTest, SourceNodeNames) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1_multinode.json"));

  // This is a bit simplistic in that it doesn't test deduplication, but it does
  // exercise a lot of the logic.
  EXPECT_THAT(
      SourceNodeNames(&config.message(), config.message().nodes()->Get(0)),
      ::testing::ElementsAreArray({"pi2"}));
  EXPECT_THAT(
      SourceNodeNames(&config.message(), config.message().nodes()->Get(1)),
      ::testing::ElementsAreArray({"pi1"}));
}

// Tests that we can deduce destination nodes from a multinode config.
TEST_F(ConfigurationTest, DestinationNodeNames) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1_multinode.json"));

  // This is a bit simplistic in that it doesn't test deduplication, but it does
  // exercise a lot of the logic.
  EXPECT_THAT(
      DestinationNodeNames(&config.message(), config.message().nodes()->Get(0)),
      ::testing::ElementsAreArray({"pi2"}));
  EXPECT_THAT(
      DestinationNodeNames(&config.message(), config.message().nodes()->Get(1)),
      ::testing::ElementsAreArray({"pi1"}));
}

// Tests that we can pull out all the nodes.
TEST_F(ConfigurationTest, GetNodes) {
  {
    FlatbufferDetachedBuffer<Configuration> config =
        ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
    const Node *pi1 = GetNode(&config.message(), "pi1");
    const Node *pi2 = GetNode(&config.message(), "pi2");

    EXPECT_THAT(GetNodes(&config.message()), ::testing::ElementsAre(pi1, pi2));
  }

  {
    FlatbufferDetachedBuffer<Configuration> config =
        ReadConfig(ArtifactPath("aos/testdata/config1.json"));
    EXPECT_THAT(GetNodes(&config.message()), ::testing::ElementsAre(nullptr));
  }
}

// Tests that we can pull out all the nodes with a tag.
TEST_F(ConfigurationTest, GetNodesWithTag) {
  {
    FlatbufferDetachedBuffer<Configuration> config =
        ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
    const Node *pi1 = GetNode(&config.message(), "pi1");
    const Node *pi2 = GetNode(&config.message(), "pi2");

    EXPECT_THAT(GetNodesWithTag(&config.message(), "a"),
                ::testing::ElementsAre(pi1));
    EXPECT_THAT(GetNodesWithTag(&config.message(), "b"),
                ::testing::ElementsAre(pi2));
    EXPECT_THAT(GetNodesWithTag(&config.message(), "c"),
                ::testing::ElementsAre(pi1, pi2));
  }

  {
    FlatbufferDetachedBuffer<Configuration> config =
        ReadConfig(ArtifactPath("aos/testdata/config1.json"));
    EXPECT_THAT(GetNodesWithTag(&config.message(), "arglfish"),
                ::testing::ElementsAre(nullptr));
  }
}

// Tests that we can check if a node has a tag.
TEST_F(ConfigurationTest, NodeHasTag) {
  {
    FlatbufferDetachedBuffer<Configuration> config =
        ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
    const Node *pi1 = GetNode(&config.message(), "pi1");
    const Node *pi2 = GetNode(&config.message(), "pi2");

    EXPECT_TRUE(NodeHasTag(pi1, "a"));
    EXPECT_FALSE(NodeHasTag(pi2, "a"));
    EXPECT_FALSE(NodeHasTag(pi1, "b"));
    EXPECT_TRUE(NodeHasTag(pi2, "b"));
    EXPECT_TRUE(NodeHasTag(pi1, "c"));
    EXPECT_TRUE(NodeHasTag(pi2, "c"));
    EXPECT_FALSE(NodeHasTag(pi1, "nope"));
    EXPECT_FALSE(NodeHasTag(pi2, "nope"));
  }

  EXPECT_TRUE(NodeHasTag(nullptr, "arglfish"));
}

// Tests that we can extract a node index from a config.
TEST_F(ConfigurationTest, GetNodeIndex) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  FlatbufferDetachedBuffer<Configuration> config2 =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  const Node *pi1 = GetNode(&config.message(), "pi1");
  const Node *pi2 = GetNode(&config.message(), "pi2");

  // Try the normal case.
  EXPECT_EQ(GetNodeIndex(&config.message(), pi1), 0);
  EXPECT_EQ(GetNodeIndex(&config.message(), pi2), 1);

  // Now try if we have node pointers from a different message.
  EXPECT_EQ(GetNodeIndex(&config2.message(), pi1), 0);
  EXPECT_EQ(GetNodeIndex(&config2.message(), pi2), 1);

  // And now try string names.
  EXPECT_EQ(GetNodeIndex(&config2.message(), pi1->name()->string_view()), 0);
  EXPECT_EQ(GetNodeIndex(&config2.message(), pi2->name()->string_view()), 1);
}

// Tests that GetNodeOrDie handles both single and multi-node worlds and returns
// valid nodes.
TEST_F(ConfigurationDeathTest, GetNodeOrDie) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  FlatbufferDetachedBuffer<Configuration> config2 =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  {
    // Simple case, nullptr -> nullptr
    FlatbufferDetachedBuffer<Configuration> single_node_config =
        ReadConfig(ArtifactPath("aos/testdata/config1.json"));
    EXPECT_EQ(nullptr, GetNodeOrDie(&single_node_config.message(), nullptr));

    // Confirm that we die when a node is passed in.
    EXPECT_DEATH(
        {
          GetNodeOrDie(&single_node_config.message(),
                       config.message().nodes()->Get(0));
        },
        "Provided a node in a single node world.");
  }

  const Node *pi1 = GetNode(&config.message(), "pi1");
  // Now try a lookup using a node from a different instance of the config.
  EXPECT_EQ(pi1,
            GetNodeOrDie(&config.message(), config2.message().nodes()->Get(0)));
}

TEST_F(ConfigurationTest, GetNodeFromHostname) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  EXPECT_EQ("pi1",
            CHECK_NOTNULL(GetNodeFromHostname(&config.message(), "raspberrypi"))
                ->name()
                ->string_view());
  EXPECT_EQ("pi2", CHECK_NOTNULL(
                       GetNodeFromHostname(&config.message(), "raspberrypi2"))
                       ->name()
                       ->string_view());
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "raspberrypi3"));
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "localhost"));
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "3"));
}

TEST_F(ConfigurationTest, GetNodeFromHostnames) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode_hostnames.json"));
  EXPECT_EQ("pi1",
            CHECK_NOTNULL(GetNodeFromHostname(&config.message(), "raspberrypi"))
                ->name()
                ->string_view());
  EXPECT_EQ("pi2", CHECK_NOTNULL(
                       GetNodeFromHostname(&config.message(), "raspberrypi2"))
                       ->name()
                       ->string_view());
  EXPECT_EQ("pi2", CHECK_NOTNULL(
                       GetNodeFromHostname(&config.message(), "raspberrypi3"))
                       ->name()
                       ->string_view());
  EXPECT_EQ("pi2",
            CHECK_NOTNULL(GetNodeFromHostname(&config.message(), "other"))
                ->name()
                ->string_view());
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "raspberrypi4"));
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "localhost"));
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "3"));
}

// Tests that SourceNodeIndex reasonably handles a multi-node log file.
TEST_F(ConfigurationTest, SourceNodeIndex) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  std::vector<size_t> result = SourceNodeIndex(&config.message());

  EXPECT_THAT(result, ::testing::ElementsAreArray({0, 1, 0, 0}));
}

// Tests that we reject invalid logging configurations.
TEST_F(ConfigurationDeathTest, InvalidLoggerConfig) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config = ReadConfig(
            ArtifactPath("aos/testdata/invalid_logging_configuration.json"));
      },
      "Logging timestamps without data");
}

// Tests that we reject duplicate timestamp destination node configurations.
TEST_F(ConfigurationDeathTest, DuplicateTimestampDestinationNodes) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config = ReadConfig(
            ArtifactPath("aos/testdata/duplicate_destination_nodes.json"));
      },
      "Found duplicate timestamp_logger_nodes in");
}

// Tests that we reject duplicate logger node configurations for a channel's
// data.
TEST_F(ConfigurationDeathTest, DuplicateLoggerNodes) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config = ReadConfig(
            ArtifactPath("aos/testdata/duplicate_logger_nodes.json"));
      },
      "Found duplicate logger_nodes in");
}

// Tests that we properly compute the queue size for the provided duration.
TEST_F(ConfigurationTest, QueueSize) {
  EXPECT_EQ(QueueSize(100, chrono::seconds(2)), 200);
  EXPECT_EQ(QueueSize(200, chrono::seconds(2)), 400);
  EXPECT_EQ(QueueSize(100, chrono::seconds(6)), 600);
  EXPECT_EQ(QueueSize(100, chrono::milliseconds(10)), 1);
  EXPECT_EQ(QueueSize(100, chrono::milliseconds(10) - chrono::nanoseconds(1)),
            1);
  EXPECT_EQ(QueueSize(100, chrono::milliseconds(10) - chrono::nanoseconds(2)),
            1);
}

// Tests that we compute scratch buffer size correctly too.
TEST_F(ConfigurationTest, QueueScratchBufferSize) {
  const aos::FlatbufferDetachedBuffer<Channel> channel =
      JsonToFlatbuffer<Channel>(
          "{ \"name\": \"/foo\", \"type\": \".aos.bar\", \"num_readers\": 5, "
          "\"num_senders\": 10 }");
  EXPECT_EQ(QueueScratchBufferSize(&channel.message()), 15);
}

// Tests that GetSchema returns schema of specified type
TEST_F(ConfigurationTest, GetSchema) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/events/pingpong_config.json"));
  FlatbufferVector<reflection::Schema> expected_schema =
      FileToFlatbuffer<reflection::Schema>(
          ArtifactPath("aos/events/ping.bfbs"));
  EXPECT_EQ(FlatbufferToJson(GetSchema(&config.message(), "aos.examples.Ping")),
            FlatbufferToJson(expected_schema));
  EXPECT_EQ(GetSchema(&config.message(), "invalid_name"), nullptr);
}

// Tests that GetSchema template returns schema of specified type
TEST_F(ConfigurationTest, GetSchemaTemplate) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/events/pingpong_config.json"));
  FlatbufferVector<reflection::Schema> expected_schema =
      FileToFlatbuffer<reflection::Schema>(
          ArtifactPath("aos/events/ping.bfbs"));
  EXPECT_EQ(FlatbufferToJson(GetSchema<aos::examples::Ping>(&config.message())),
            FlatbufferToJson(expected_schema));
}

// Tests that GetSchemaDetachedBuffer returns detached buffer of specified type
TEST_F(ConfigurationTest, GetSchemaDetachedBuffer) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/events/pingpong_config.json"));
  FlatbufferVector<reflection::Schema> expected_schema =
      FileToFlatbuffer<reflection::Schema>(
          ArtifactPath("aos/events/ping.bfbs"));
  EXPECT_EQ(FlatbufferToJson(
                GetSchemaDetachedBuffer(&config.message(), "aos.examples.Ping")
                    .value()),
            FlatbufferToJson(expected_schema));
  EXPECT_EQ(GetSchemaDetachedBuffer(&config.message(), "invalid_name"),
            std::nullopt);
}

}  // namespace testing
}  // namespace configuration
}  // namespace aos
