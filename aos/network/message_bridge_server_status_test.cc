#include "aos/network/message_bridge_server_status.h"

#include "gtest/gtest.h"

#include "aos/events/simulated_event_loop.h"

namespace aos::message_bridge::testing {

TEST(MessageBridgeServerStatus, NoThrowOnInvalidServerNode) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config(
      aos::configuration::ReadConfig(
          "message_bridge_test_combined_timestamps_common_config.json"));
  aos::SimulatedEventLoopFactory factory(&config.message());
  // Configure the server node to be `pi1` - for details
  // on the configuration, refer to
  // `message_bridge_test_combined_timestamps_common.json`.
  std::unique_ptr<EventLoop> event_loop =
      factory.GetNodeEventLoopFactory("pi1")->MakeEventLoop("test");
  MessageBridgeServerStatus server_status(event_loop.get());
  // We want to choose a client node such that there is no server for that
  // client on this node. A simple way to do this is to choose the client node
  // to be the same as the server node. There will never be a valid `NodeState`
  // object assigned in `MessageBridgeServerStatus::nodes_`, which is an
  // `std::vector` of `std::optional<NodeState> elements`. This is because a
  // node will not be allowed to forward messages to itself since that would
  // cause a loop of the same message being forwarded over-and-over again. We're
  // making use of this property to simulate a multi-node software update
  // scenario in which one node was upgraded to a config that had a valid
  // connection to another node and started forwarding messages to the other
  // node. Since the other node was in the process of being updated to the new
  // software, it did not have the updated config yet, and couldn't find a
  // server node corresponding to the client node. In this situation,
  // `MaybeIncrementInvalidConnectionCount()` ended-up accessing an
  // `std::optional` that was unset. As a regression test, we want to ensure
  // that no exceptions are raised in this scenario, now that the proper checks
  // have been added.
  const aos::Node *client_node =
      aos::configuration::GetNode(&config.message(), "pi1");
  EXPECT_NE(client_node, nullptr);
  EXPECT_NO_THROW(
      server_status.MaybeIncrementInvalidConnectionCount(client_node));
}

}  // namespace aos::message_bridge::testing