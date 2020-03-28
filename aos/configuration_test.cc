#include "aos/configuration.h"

#include "absl/strings/strip.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/test_logging.h"
#include "aos/util/file.h"
#include "flatbuffers/reflection.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace aos {
namespace configuration {
namespace testing {

const std::string kConfigPrefix = "aos/testdata/";

class ConfigurationTest : public ::testing::Test {
 public:
  ConfigurationTest() { ::aos::testing::EnableTestLogging(); }
};

typedef ConfigurationTest ConfigurationDeathTest;

// *the* expected location for all working tests.
const char *kExpectedLocation =
    "{ \"name\": \"/foo\", \"type\": \".aos.bar\", \"max_size\": 5 }";
// And for multinode setups
const char *kExpectedMultinodeLocation =
    "{ \"name\": \"/foo\", \"type\": \".aos.bar\", \"max_size\": 5, \"source_node\": \"pi1\" }";

// Tests that we can read and merge a configuration.
TEST_F(ConfigurationTest, ConfigMerge) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(kConfigPrefix + "config1.json");
  LOG(INFO) << "Read: " << FlatbufferToJson(config, true);

  EXPECT_EQ(
      absl::StripSuffix(
          util::ReadFileToStringOrDie(kConfigPrefix + "expected.json"), "\n"),
      FlatbufferToJson(config, true));
}

// Tests that we can get back a ChannelIndex.
TEST_F(ConfigurationTest, ChannelIndex) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(kConfigPrefix + "config1.json");

  EXPECT_EQ(
      ChannelIndex(&config.message(), config.message().channels()->Get(1u)),
      1u);
}

// Tests that we can read and merge a multinode configuration.
TEST_F(ConfigurationTest, ConfigMergeMultinode) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(kConfigPrefix + "config1_multinode.json");
  LOG(INFO) << "Read: " << FlatbufferToJson(config, true);

  EXPECT_EQ(
      std::string(absl::StripSuffix(
          util::ReadFileToStringOrDie(kConfigPrefix + "expected_multinode.json"),
          "\n")),
      FlatbufferToJson(config, true));
}

// Tests that we sort the entries in a config so we can look entries up.
TEST_F(ConfigurationTest, UnsortedConfig) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(kConfigPrefix + "backwards.json");

  LOG(INFO) << "Read: " << FlatbufferToJson(config, true);

  EXPECT_EQ(FlatbufferToJson(GetChannel(config, ".aos.robot_state",
                                        "aos.RobotState", "app1", nullptr)),
            "{ \"name\": \".aos.robot_state\", \"type\": \"aos.RobotState\", "
            "\"max_size\": 5 }");
}

// Tests that we die when a file is imported twice.
TEST_F(ConfigurationDeathTest, DuplicateFile) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(kConfigPrefix + "config1_bad.json");
      },
      kConfigPrefix + "config1_bad.json");
}

// Tests that we can modify a config with a json snippet.
TEST_F(ConfigurationTest, MergeWithConfig) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(kConfigPrefix + "config1.json");
  LOG(INFO) << "Read: " << FlatbufferToJson(config, true);

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

  EXPECT_EQ(
      absl::StripSuffix(util::ReadFileToStringOrDie(
                            kConfigPrefix + "expected_merge_with.json"),
                        "\n"),
      FlatbufferToJson(updated_config, true));
}

// Tests that we can lookup a location, complete with maps, from a merged
// config.
TEST_F(ConfigurationTest, GetChannel) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(kConfigPrefix + "config1.json");

  // Test a basic lookup first.
  EXPECT_EQ(
      FlatbufferToJson(GetChannel(config, "/foo", ".aos.bar", "app1", nullptr)),
      kExpectedLocation);

  // Test that an invalid name results in nullptr back.
  EXPECT_EQ(GetChannel(config, "/invalid_name", ".aos.bar", "app1", nullptr),
            nullptr);

  // Tests that a root map/rename works. And that they get processed from the
  // bottom up.
  EXPECT_EQ(FlatbufferToJson(
                GetChannel(config, "/batman", ".aos.bar", "app1", nullptr)),
            kExpectedLocation);

  // And then test that an application specific map/rename works.
  EXPECT_EQ(
      FlatbufferToJson(GetChannel(config, "/bar", ".aos.bar", "app1", nullptr)),
      kExpectedLocation);
  EXPECT_EQ(
      FlatbufferToJson(GetChannel(config, "/baz", ".aos.bar", "app2", nullptr)),
      kExpectedLocation);

  // And then test that an invalid application name gets properly ignored.
  EXPECT_EQ(
      FlatbufferToJson(GetChannel(config, "/foo", ".aos.bar", "app3", nullptr)),
      kExpectedLocation);
}

// Tests that we can lookup a location with node specific maps.
TEST_F(ConfigurationTest, GetChannelMultinode) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(kConfigPrefix + "good_multinode.json");
  const Node *pi1 = GetNode(&config.message(), "pi1");
  const Node *pi2 = GetNode(&config.message(), "pi2");

  // Test a basic lookup first.
  EXPECT_EQ(
      FlatbufferToJson(GetChannel(config, "/foo", ".aos.bar", "app1", pi1)),
      kExpectedMultinodeLocation);
  EXPECT_EQ(
      FlatbufferToJson(GetChannel(config, "/foo", ".aos.bar", "app1", pi2)),
      kExpectedMultinodeLocation);

  // Tests that a root map/rename works with a node specific map.
  EXPECT_EQ(FlatbufferToJson(
                GetChannel(config, "/batman", ".aos.bar", "app1", pi1)),
            kExpectedMultinodeLocation);

  // Tests that a root map/rename fails with a node specific map for the wrong
  // node.
  EXPECT_EQ(GetChannel(config, "/batman", ".aos.bar", "app1", pi2), nullptr);

  // And then test that an application specific map/rename works.
  EXPECT_EQ(
      FlatbufferToJson(GetChannel(config, "/batman2", ".aos.bar", "app1", pi1)),
      kExpectedMultinodeLocation);
  EXPECT_EQ(
      FlatbufferToJson(GetChannel(config, "/batman3", ".aos.bar", "app1", pi1)),
      kExpectedMultinodeLocation);

  // And then that it fails when the node changes.
  EXPECT_EQ(GetChannel(config, "/batman3", ".aos.bar", "app1", pi2), nullptr);
}

// Tests that we can lookup a location with type specific maps.
TEST_F(ConfigurationTest, GetChannelTypedMultinode) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(kConfigPrefix + "good_multinode.json");
  const Node *pi1 = GetNode(&config.message(), "pi1");

  // Test a basic lookup first.
  EXPECT_EQ(
      FlatbufferToJson(GetChannel(config, "/batman", ".aos.bar", "app1", pi1)),
      kExpectedMultinodeLocation);

  // Now confirm that a second message on the same name doesn't get remapped.
  const char *kExpectedBazMultinodeLocation =
      "{ \"name\": \"/batman\", \"type\": \".aos.baz\", \"max_size\": 5, "
      "\"source_node\": \"pi1\" }";
  EXPECT_EQ(
      FlatbufferToJson(GetChannel(config, "/batman", ".aos.baz", "app1", pi1)),
      kExpectedBazMultinodeLocation);
}

// Tests that we reject a configuration which has a nodes list, but has channels
// withoout source_node filled out.
TEST_F(ConfigurationDeathTest, InvalidSourceNode) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(kConfigPrefix + "invalid_nodes.json");
      },
      "source_node");

  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(kConfigPrefix + "invalid_source_node.json");
      },
      "source_node");

  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(kConfigPrefix + "invalid_destination_node.json");
      },
      "destination_nodes");

  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(kConfigPrefix + "self_forward.json");
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

  FlatbufferDetachedBuffer<Channel> logged_on_both_channel (
      JsonToFlatbuffer(
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

  // No logger.
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&not_logged_channel.message(),
                                            &foo_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&not_logged_channel.message(),
                                           &bar_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&not_logged_channel.message(),
                                            &baz_node.message()));

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

  FlatbufferDetachedBuffer<Channel> logged_on_both_channel (
      JsonToFlatbuffer(
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
      ReadConfig(kConfigPrefix + "config1_multinode.json");

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
      ReadConfig(kConfigPrefix + "config1_multinode.json");

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
        ReadConfig(kConfigPrefix + "good_multinode.json");
    const Node *pi1 = GetNode(&config.message(), "pi1");
    const Node *pi2 = GetNode(&config.message(), "pi2");

    EXPECT_THAT(GetNodes(&config.message()), ::testing::ElementsAre(pi1, pi2));
  }

  {
    FlatbufferDetachedBuffer<Configuration> config =
        ReadConfig(kConfigPrefix + "config1.json");
    EXPECT_THAT(GetNodes(&config.message()), ::testing::ElementsAre(nullptr));
  }
}

// Tests that we can extract a node index from a config.
TEST_F(ConfigurationTest, GetNodeIndex) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(kConfigPrefix + "good_multinode.json");
  FlatbufferDetachedBuffer<Configuration> config2 =
      ReadConfig(kConfigPrefix + "good_multinode.json");
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
      ReadConfig(kConfigPrefix + "good_multinode.json");
  FlatbufferDetachedBuffer<Configuration> config2 =
      ReadConfig(kConfigPrefix + "good_multinode.json");
  {
    // Simple case, nullptr -> nullptr
    FlatbufferDetachedBuffer<Configuration> single_node_config =
        ReadConfig(kConfigPrefix + "config1.json");
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
      ReadConfig(kConfigPrefix + "good_multinode.json");
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
      ReadConfig(kConfigPrefix + "good_multinode_hostnames.json");
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
  EXPECT_EQ("pi2", CHECK_NOTNULL(
                       GetNodeFromHostname(&config.message(), "other"))
                       ->name()
                       ->string_view());
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "raspberrypi4"));
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "localhost"));
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "3"));
}

}  // namespace testing
}  // namespace configuration
}  // namespace aos
