#include "absl/types/span.h"

#include "aos/events/event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/network/web_proxy_generated.h"

namespace aos {
namespace web_proxy {

int GetPacketCount(const Context &context);

// Packs a message embedded in context into a MessageHeader on fbb. Handles
// multipart messages by use of the packet_index.
flatbuffers::Offset<MessageHeader> PackMessage(
    flatbuffers::FlatBufferBuilder *fbb, const Context &context,
    int channel_index, int packet_index);

// Returns the size that the overall packed message packed by PackMessage will
// be for the provided packet index.
size_t PackedMessageSize(const Context &context, int packet_index);

// Packs the provided raw data into a series of MessageHeader's of the
// appropriate length.
std::vector<FlatbufferDetachedBuffer<MessageHeader>> PackBuffer(
    absl::Span<const uint8_t> span);

}  // namespace web_proxy
}  // namespace aos
