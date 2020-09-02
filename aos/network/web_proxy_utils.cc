#include "aos/network/web_proxy_utils.h"

namespace aos {
namespace web_proxy {

namespace {
// Recommended max size is 64KiB for compatibility reasons. 256KiB theoretically
// works on chrome but seemed to have some consistency issues. Picked a size in
// the middle which seems to work.
constexpr size_t kPacketSize = 125000;

int GetPacketCountFromSize(const int packet_size) {
  return packet_size / kPacketSize + 1;
}

flatbuffers::Offset<flatbuffers::Vector<uint8_t>> FillOutPacketVector(
    flatbuffers::FlatBufferBuilder *fbb, absl::Span<const uint8_t> span,
    const int packet_index) {
  flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset;
  if (kPacketSize * (packet_index + 1) < span.size()) {
    data_offset = fbb->CreateVector(
        static_cast<const uint8_t *>(span.data()) + kPacketSize * packet_index,
        kPacketSize);
  } else {
    const int prefix_size = kPacketSize * packet_index;
    data_offset = fbb->CreateVector(
        static_cast<const uint8_t *>(span.data()) + prefix_size,
        span.size() - prefix_size);
  }
  return data_offset;
}
}  // namespace

int GetPacketCount(const Context &context) {
  return GetPacketCountFromSize(context.size);
}

flatbuffers::Offset<MessageHeader> PackMessage(
    flatbuffers::FlatBufferBuilder *fbb, const Context &context,
    int channel_index, int packet_index) {
  const int packet_count = GetPacketCount(context);
  const flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
      FillOutPacketVector(
          fbb,
          absl::Span<const uint8_t>{static_cast<const uint8_t *>(context.data),
                                    context.size},
          packet_index);

  MessageHeader::Builder message_header_builder(*fbb);
  message_header_builder.add_channel_index(channel_index);
  message_header_builder.add_queue_index(context.queue_index);
  message_header_builder.add_packet_count(packet_count);
  message_header_builder.add_packet_index(packet_index);
  message_header_builder.add_data(data_offset);
  message_header_builder.add_length(context.size);
  message_header_builder.add_monotonic_sent_time(
      context.monotonic_event_time.time_since_epoch().count());

  return message_header_builder.Finish();
}

std::vector<FlatbufferDetachedBuffer<MessageHeader>> PackBuffer(
    absl::Span<const uint8_t> span) {
  flatbuffers::FlatBufferBuilder fbb;
  std::vector<FlatbufferDetachedBuffer<MessageHeader>> buffers;
  const int packet_count = GetPacketCountFromSize(span.size());
  for (int ii = 0; ii < packet_count; ++ii) {
    const flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
        FillOutPacketVector(&fbb, span, ii);

    MessageHeader::Builder message_header_builder(fbb);
    message_header_builder.add_packet_count(packet_count);
    message_header_builder.add_packet_index(ii);
    message_header_builder.add_data(data_offset);
    message_header_builder.add_length(span.size());

    fbb.Finish(message_header_builder.Finish());

    buffers.emplace_back(fbb.Release());
  }
  return buffers;
}

}  // namespace web_proxy
}  // namespace aos
