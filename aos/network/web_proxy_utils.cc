#include "aos/network/web_proxy_utils.h"

namespace aos {
namespace web_proxy {

// Recommended max size is 64KiB for compatibility reasons. 256KiB theoretically
// works on chrome but seemed to have some consistency issues. Picked a size in
// the middle which seems to work.
constexpr size_t kPacketSize = 125000;

int GetPacketCount(const Context &context) {
  return context.size / kPacketSize + 1;
}

flatbuffers::Offset<MessageHeader> PackMessage(
    flatbuffers::FlatBufferBuilder *fbb, const Context &context,
    int channel_index, int packet_index) {
  int packet_count = GetPacketCount(context);
  flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset;
  if (kPacketSize * (packet_index + 1) < context.size) {
    data_offset = fbb->CreateVector(
        static_cast<const uint8_t *>(context.data) + kPacketSize * packet_index,
        kPacketSize);
  } else {
    int prefix_size = kPacketSize * packet_index;
    data_offset = fbb->CreateVector(
        static_cast<const uint8_t *>(context.data) + prefix_size,
        context.size - prefix_size);
  }

  MessageHeader::Builder message_header_builder(*fbb);
  message_header_builder.add_channel_index(channel_index);
  message_header_builder.add_queue_index(context.queue_index);
  message_header_builder.add_packet_count(packet_count);
  message_header_builder.add_packet_index(packet_index);
  message_header_builder.add_data(data_offset);
  message_header_builder.add_length(context.size);

  return message_header_builder.Finish();
}

}  // namespace web_proxy
}  // namespace aos
