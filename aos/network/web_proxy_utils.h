#include "aos/network/web_proxy_generated.h"
#include "aos/events/event_loop.h"

namespace aos {
namespace web_proxy {

int GetPacketCount(const Context &context);

/*
 * Packs a message embedded in context into a MessageHeader on fbb. Handles
 * multipart messages by use of the packet_index.
 * TODO(alex): make this an iterator that returns each packet sequentially
 */
flatbuffers::Offset<MessageHeader> PackMessage(
    flatbuffers::FlatBufferBuilder *fbb, const Context &context,
    int channel_index, int packet_index);


}  // namespace web_proxy
}  // namespace aos
