#include "aos/network/rawrtc.h"

extern "C" {
#include <rawrtc.h>

#include "rawrtcc/utils.h"
}

#include <functional>
#include <string>

#include "flatbuffers/flatbuffers.h"
#include "glog/logging.h"

DEFINE_int32(min_ice_port, -1,
             "Minimum port number to use for ICE candidates.");
DEFINE_int32(max_ice_port, -1,
             "Maximum port number to use for ICE candidates.");

namespace aos {
namespace web_proxy {
namespace {
enum {
  TRANSPORT_BUFFER_LENGTH = 1048576,  // 1 MiB
};
}

ScopedDataChannel::ScopedDataChannel() {}

std::shared_ptr<ScopedDataChannel> ScopedDataChannel::MakeDataChannel() {
  std::shared_ptr<ScopedDataChannel> channel(new ScopedDataChannel());
  channel->self_ = channel;
  return channel;
}

void ScopedDataChannel::Open(struct rawrtc_peer_connection *connection,
                             const std::string &label) {
  label_ = label;
  VLOG(1) << "(" << this << ")  Opening " << label_;
  struct rawrtc_data_channel_parameters *channel_parameters;
  // Create data channel parameters
  // TODO(austin): TYPE?
  CHECK_RAWRTC(rawrtc_data_channel_parameters_create(
      &channel_parameters, label.c_str(),
      RAWRTC_DATA_CHANNEL_TYPE_RELIABLE_ORDERED, 0, NULL, false, 0));

  // Create data channel
  CHECK_RAWRTC(rawrtc_peer_connection_create_data_channel(
      &data_channel_, connection, channel_parameters,
      StaticDataChannelOpenHandler, StaticBufferedAmountLowHandler,
      StaticDataChannelErrorHandler, StaticDataChannelCloseHandler,
      StaticDataChannelMessageHandler, this));

  // Un-reference data channel parameters
  mem_deref(channel_parameters);
}

void ScopedDataChannel::Open(struct rawrtc_data_channel *const channel) {
  struct rawrtc_data_channel_parameters *parameters;
  enum rawrtc_code const ignore[] = {RAWRTC_CODE_NO_VALUE};
  char *label = NULL;

  // Get data channel label and protocol
  CHECK_RAWRTC(rawrtc_data_channel_get_parameters(&parameters, channel));
  CHECK_RAWRTC_IGNORE(
      rawrtc_data_channel_parameters_get_label(&label, parameters), ignore);
  if (label) {
    label_ = label;
    mem_deref(label);
  }
  mem_deref(parameters);

  VLOG(1) << "(" << this << ") New data channel instance: " << label_;

  mem_ref(channel);
  data_channel_ = channel;

  CHECK_RAWRTC(rawrtc_data_channel_set_arg(data_channel_, this));

  CHECK_RAWRTC(rawrtc_data_channel_set_open_handler(
      data_channel_, StaticDataChannelOpenHandler));

  CHECK_RAWRTC(rawrtc_data_channel_set_buffered_amount_low_handler(
      data_channel_, StaticBufferedAmountLowHandler));

  CHECK_RAWRTC(rawrtc_data_channel_set_error_handler(
      data_channel_, StaticDataChannelErrorHandler));

  CHECK_RAWRTC(rawrtc_data_channel_set_close_handler(
      data_channel_, StaticDataChannelCloseHandler));

  CHECK_RAWRTC(rawrtc_data_channel_set_message_handler(
      data_channel_, StaticDataChannelMessageHandler));
}

ScopedDataChannel::~ScopedDataChannel() {
  CHECK(opened_);
  CHECK(closed_) << ": Never closed " << label();
  CHECK(data_channel_ == nullptr)
      << ": Destroying open data channel " << this << ".";
}

void ScopedDataChannel::StaticDataChannelOpenHandler(void *const arg) {
  ScopedDataChannel *const client = reinterpret_cast<ScopedDataChannel *>(arg);
  CHECK(!client->opened_);
  CHECK(!client->closed_);
  if (client->on_open_) client->on_open_();
  client->opened_ = true;
}

void ScopedDataChannel::StaticBufferedAmountLowHandler(void *const arg) {
  ScopedDataChannel *const client = reinterpret_cast<ScopedDataChannel *>(arg);
  if (client->on_buffered_amount_low_) client->on_buffered_amount_low_();
}

void ScopedDataChannel::StaticDataChannelErrorHandler(void *const arg) {
  ScopedDataChannel *const client = reinterpret_cast<ScopedDataChannel *>(arg);
  if (client->on_error_) client->on_error_();
}

void ScopedDataChannel::StaticDataChannelCloseHandler(void *const arg) {
  ScopedDataChannel *const client = reinterpret_cast<ScopedDataChannel *>(arg);
  CHECK(client->opened_);
  CHECK(!client->closed_);
  // Close() assumes that this method will do the final cleanup.  The destructor
  // CHECKs that.
  client->closed_ = true;
  struct rawrtc_data_channel *data_channel = client->data_channel_;
  client->data_channel_ = nullptr;
  if (client->on_close_) {
    // Take the function so we can call it without referencing client.
    // This could destroy the client when the function is deleted by releasing
    // any shared_ptrs.
    std::function<void()> on_close = std::move(client->on_close_);
    on_close();
  }
  mem_deref(data_channel);
  client->self_.reset();
}

void ScopedDataChannel::StaticDataChannelMessageHandler(
    struct mbuf *const
        buffer,  // nullable (in case partial delivery has been requested)
    enum rawrtc_data_channel_message_flag const flags, void *const arg) {
  ScopedDataChannel *const client = reinterpret_cast<ScopedDataChannel *>(arg);
  if (client->on_message_) client->on_message_(buffer, flags);
}

void ScopedDataChannel::Close() {
  CHECK(opened_);
  if (!closed_) {
    CHECK_RAWRTC(rawrtc_data_channel_close(data_channel_));
  }
}

void ScopedDataChannel::Send(const ::flatbuffers::DetachedBuffer &buffer) {
  struct mbuf *mbuffer = mbuf_alloc(buffer.size());
  mbuf_write_mem(mbuffer, buffer.data(), buffer.size());
  mbuf_set_pos(mbuffer, 0);

  Send(mbuffer);

  mem_deref(mbuffer);
}

void ScopedDataChannel::Send(struct mbuf *buffer) {
  // TODO(austin): Checking isn't right, handle errors more gracefully.
  CHECK_RAWRTC(
      rawrtc_data_channel_send(CHECK_NOTNULL(data_channel_), buffer, true));
}

uint64_t ScopedDataChannel::buffered_amount() {
  return 0;

  // TODO(austin): Not implemented yet...
  uint64_t result;
  CHECK_RAWRTC(rawrtc_data_channel_get_buffered_amount(
      &result, CHECK_NOTNULL(data_channel_)));
  return result;
}

RawRTCConnection::RawRTCConnection() {}

void RawRTCConnection::Open() {
  const char *const stun_google_com_urls[] = {"stun:stun.l.google.com:19302",
                                              "stun:stun1.l.google.com:19302"};

  struct rawrtc_peer_connection_configuration *configuration = nullptr;

  CHECK_RAWRTC(rawrtc_peer_connection_configuration_create(
      &configuration, RAWRTC_ICE_GATHER_POLICY_ALL));

  // Add ICE servers to configuration
  CHECK_RAWRTC(rawrtc_peer_connection_configuration_add_ice_server(
      configuration, stun_google_com_urls, ARRAY_SIZE(stun_google_com_urls),
      NULL, NULL, RAWRTC_ICE_CREDENTIAL_TYPE_NONE));

  // Set the SCTP transport's buffer length
  CHECK_RAWRTC(rawrtc_peer_connection_configuration_set_sctp_buffer_length(
      configuration, TRANSPORT_BUFFER_LENGTH, TRANSPORT_BUFFER_LENGTH));

  if (FLAGS_min_ice_port >= 0 && FLAGS_max_ice_port >= 0) {
    CHECK_LT(FLAGS_min_ice_port, FLAGS_max_ice_port);
    // Set the port range to use for ICE candidates.
    CHECK_RAWRTC(rawrtc_peer_connection_configuration_set_ice_udp_port_range(
        configuration, FLAGS_min_ice_port, FLAGS_max_ice_port));
  }

  // Create peer connection
  CHECK_RAWRTC(rawrtc_peer_connection_create(
      &connection_, configuration, StaticNegotiationNeededHandler,
      StaticLocalCandidateHandler,
      StaticPeerConnectionLocalCandidateErrorHandler,
      StaticSignalingStateChangeHandler, StaticIceTransportStateChangeHandler,
      StaticIceGathererStateChangeHandler, StaticConnectionStateChangeHandler,
      StaticDataChannelHandler, this));

  mem_deref(configuration);
}

RawRTCConnection::~RawRTCConnection() {
  CHECK_RAWRTC(rawrtc_peer_connection_close(connection_));
  mem_deref(connection_);
  connection_ = nullptr;
}

void RawRTCConnection::StaticNegotiationNeededHandler(void *const arg) {
  RawRTCConnection *const client = reinterpret_cast<RawRTCConnection *>(arg);
  if (client->on_negotiation_needed_) client->on_negotiation_needed_();
}

void RawRTCConnection::StaticLocalCandidateHandler(
    struct rawrtc_peer_connection_ice_candidate *const candidate,
    char const *const url,  // read-only
    void *const arg) {
  RawRTCConnection *const client = reinterpret_cast<RawRTCConnection *>(arg);
  if (client->on_local_candidate_) client->on_local_candidate_(candidate, url);
}

void RawRTCConnection::StaticPeerConnectionLocalCandidateErrorHandler(
    struct rawrtc_peer_connection_ice_candidate *const candidate,
    char const *const url, uint16_t const error_code,
    char const *const error_text, void *const arg) {
  RawRTCConnection *const client = reinterpret_cast<RawRTCConnection *>(arg);
  LOG(ERROR) << "(" << client << ") ICE candidate error, URL: " << url
             << ", reason: " << error_text;
  if (client->on_peer_connection_local_candidate_error_)
    client->on_peer_connection_local_candidate_error_(candidate, url,
                                                      error_code, error_text);
}

void RawRTCConnection::StaticSignalingStateChangeHandler(
    const enum rawrtc_signaling_state state, void *const arg) {
  RawRTCConnection *const client = reinterpret_cast<RawRTCConnection *>(arg);
  VLOG(1) << "(" << client << ") Signaling state change: "
          << rawrtc_signaling_state_to_name(state);
  if (client->on_signaling_state_change_)
    client->on_signaling_state_change_(state);
}

void RawRTCConnection::StaticIceTransportStateChangeHandler(
    const enum rawrtc_ice_transport_state state, void *const arg) {
  RawRTCConnection *const client = reinterpret_cast<RawRTCConnection *>(arg);
  VLOG(1) << "(" << client << ") ICE transport state: "
          << rawrtc_ice_transport_state_to_name(state);
  if (client->on_ice_transport_state_change_)
    client->on_ice_transport_state_change_(state);
}

void RawRTCConnection::StaticIceGathererStateChangeHandler(
    const enum rawrtc_ice_gatherer_state state, void *const arg) {
  RawRTCConnection *const client = reinterpret_cast<RawRTCConnection *>(arg);
  VLOG(1) << "(" << client << ") ICE gatherer state: "
          << rawrtc_ice_gatherer_state_to_name(state);
  if (client->on_ice_gatherer_state_change_)
    client->on_ice_gatherer_state_change_(state);
}

void RawRTCConnection::StaticConnectionStateChangeHandler(
    const enum rawrtc_peer_connection_state state,  // read-only
    void *const arg) {
  RawRTCConnection *const client = reinterpret_cast<RawRTCConnection *>(arg);
  VLOG(1) << "(" << client << ") Peer connection state change: "
          << rawrtc_peer_connection_state_to_name(state);
  if (client->on_connection_state_change_)
    client->on_connection_state_change_(state);
}

void RawRTCConnection::StaticDataChannelHandler(
    struct rawrtc_data_channel
        *const channel,  // read-only, MUST be referenced when used
    void *const arg) {
  RawRTCConnection *const client = reinterpret_cast<RawRTCConnection *>(arg);
  if (client->on_data_channel_) {
    std::shared_ptr<ScopedDataChannel> new_channel =
        ScopedDataChannel::MakeDataChannel();
    new_channel->Open(channel);
    client->on_data_channel_(std::move(new_channel));
  }
}

}  // namespace web_proxy
}  // namespace aos
