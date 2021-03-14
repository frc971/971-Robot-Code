#include "aos/network/message_bridge_protocol.h"

#include <string_view>

#include "aos/configuration.h"
#include "aos/flatbuffer_merge.h"
#include "aos/flatbuffers.h"
#include "aos/network/connect_generated.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {
namespace message_bridge {

aos::FlatbufferDetachedBuffer<aos::message_bridge::Connect> MakeConnectMessage(
    const Configuration *config, const Node *my_node,
    std::string_view remote_name, const UUID &boot_uuid) {
  CHECK(config->has_nodes()) << ": Config must have nodes to transfer.";

  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  flatbuffers::Offset<flatbuffers::String> boot_uuid_offset =
      boot_uuid.PackString(&fbb);

  flatbuffers::Offset<Node> node_offset =
      RecursiveCopyFlatBuffer<Node>(my_node, &fbb);
  const std::string_view node_name = my_node->name()->string_view();

  std::vector<flatbuffers::Offset<Channel>> channel_offsets;
  for (const Channel *channel : *config->channels()) {
    if (channel->has_destination_nodes()) {
      for (const Connection *connection : *channel->destination_nodes()) {
        if (connection->name()->string_view() == node_name &&
            channel->source_node()->string_view() == remote_name) {
          // Remove the schema to save some space on the wire.
          aos::FlatbufferDetachedBuffer<Channel> cleaned_channel =
              RecursiveCopyFlatBuffer<Channel>(channel);
          cleaned_channel.mutable_message()->clear_schema();
          channel_offsets.emplace_back(
              CopyFlatBuffer<Channel>(&cleaned_channel.message(), &fbb));
        }
      }
    }
  }

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Channel>>>
      channels_offset = fbb.CreateVector(channel_offsets);

  Connect::Builder connect_builder(fbb);
  connect_builder.add_channels_to_transfer(channels_offset);
  connect_builder.add_node(node_offset);
  connect_builder.add_boot_uuid(boot_uuid_offset);
  fbb.Finish(connect_builder.Finish());

  return fbb.Release();
}

}  // namespace message_bridge
}  // namespace aos
