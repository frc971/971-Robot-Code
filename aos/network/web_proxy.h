#ifndef AOS_NETWORK_WEB_PROXY_H_
#define AOS_NETWORK_WEB_PROXY_H_

#include <deque>
#include <map>
#include <set>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/mutex/mutex.h"
#include "aos/network/connect_generated.h"
#include "aos/network/rawrtc.h"
#include "aos/network/web_proxy_generated.h"
#include "aos/seasocks/seasocks_logger.h"
#include "flatbuffers/flatbuffers.h"
#include "seasocks/Server.h"
#include "seasocks/StringUtil.h"
#include "seasocks/WebSocket.h"

namespace aos {
namespace web_proxy {

class Connection;
class Subscriber;
class ApplicationConnection;

enum class StoreHistory {
  kNo,
  kYes,
};

// Basic class that handles receiving new websocket connections. Creates a new
// Connection to manage the rest of the negotiation and data passing. When the
// websocket closes, it deletes the Connection.
class WebsocketHandler : public ::seasocks::WebSocket::Handler {
 public:
  WebsocketHandler(::seasocks::Server *server, aos::EventLoop *event_loop,
                   StoreHistory store_history,
                   int per_channel_buffer_size_bytes);
  void onConnect(::seasocks::WebSocket *sock) override;
  void onData(::seasocks::WebSocket *sock, const uint8_t *data,
              size_t size) override;
  void onDisconnect(::seasocks::WebSocket *sock) override;
  // Stops recording data, even if the event loop continues running. This allows
  // us to continue serving the webserver + websocket server, without having to
  // load more actual data.
  void StopRecording() { recording_ = false; }

 private:
  ::seasocks::Server *server_;
  std::vector<std::unique_ptr<Subscriber>> subscribers_;
  const aos::FlatbufferDetachedBuffer<aos::Configuration> config_;

  std::map<::seasocks::WebSocket *, std::unique_ptr<ApplicationConnection>>
      connections_;

  EventLoop *const event_loop_;
  // Whether to pay attention to new messages.
  bool recording_ = true;
};

// Wrapper class that manages the seasocks server and WebsocketHandler.
class WebProxy {
 public:
  // Constructs a WebProxy object for interacting with a webpage. store_history
  // and per_channel_buffer_size_bytes specify how we manage delivering LOSSLESS
  // messages to the client:
  // * store_history specifies whether we should always buffer up data for all
  //   channels--even for messages that are played prior to the client
  //   connecting. This is mostly useful for log replay where the client
  //   will typically connect after the logfile has been fully loaded/replayed.
  // * per_channel_buffer_size_bytes is the maximum amount of data to buffer
  //   up per channel (-1 will indicate infinite data, which is used during log
  //   replay). This is divided by the max_size per channel to determine
  //   how many messages to queue up.
  WebProxy(aos::EventLoop *event_loop, StoreHistory store_history,
           int per_channel_buffer_size_bytes);
  WebProxy(aos::ShmEventLoop *event_loop, StoreHistory store_history,
           int per_channel_buffer_size_bytes);
  WebProxy(aos::EventLoop *event_loop, aos::internal::EPoll *epoll,
           StoreHistory store_history, int per_channel_buffer_size_bytes);
  ~WebProxy();

  void SetDataPath(const char *path) { server_.setStaticPath(path); }

  // Stops recording data. Useful for setting end times in log replay.
  void StopRecording();

 private:
  aos::internal::EPoll internal_epoll_;
  aos::internal::EPoll *const epoll_;
  ::seasocks::Server server_;
  std::shared_ptr<WebsocketHandler> websocket_handler_;
};

// Seasocks requires that sends happen on the correct thread. This class takes a
// detached buffer to send on a specific websocket connection and sends it when
// seasocks is ready.
class UpdateData : public ::seasocks::Server::Runnable {
 public:
  UpdateData(::seasocks::WebSocket *websocket,
             flatbuffers::DetachedBuffer &&buffer)
      : sock_(websocket), buffer_(std::move(buffer)) {}
  ~UpdateData() override = default;
  UpdateData(const UpdateData &) = delete;
  UpdateData &operator=(const UpdateData &) = delete;

  void run() override { sock_->send(buffer_.data(), buffer_.size()); }

 private:
  ::seasocks::WebSocket *sock_;
  const flatbuffers::DetachedBuffer buffer_;
};

// Represents a fetcher and all the Connections that care about it.
// Handles building the message and telling each connection to send it.
// indexed by location of the channel it handles in the config.
// Subscriber also uses an internal buffer to store past messages. This is
// primarily meant for use in offline log replay/simulation where we want to be
// able to store infinite buffers. In the future, we will probably want to be
// able to specify *which* channels to store buffers for so that we aren't just
// loading the entire logfile into memory.
class Subscriber {
 public:
  Subscriber(std::unique_ptr<RawFetcher> fetcher, int channel_index,
             StoreHistory store_history, int buffer_size)
      : fetcher_(std::move(fetcher)),
        channel_index_(channel_index),
        store_history_(store_history == StoreHistory::kYes),
        buffer_size_(buffer_size) {}

  // Runs a single iteration of going through and fetching new data as needed
  // and servicing any WebRTC channels that are requesting messages.
  // fetch_new specifies whether we should actually attempt to retrieve new data
  // on the channel--if false, will only worry about sending existing data to
  // any clients.
  void RunIteration(bool fetch_new);

  void AddListener(std::shared_ptr<ScopedDataChannel> data_channel,
                   TransferMethod transfer_method);

  void RemoveListener(std::shared_ptr<ScopedDataChannel> data_channel);

 private:
  struct ChannelInformation {
    TransferMethod transfer_method;
    // Queue index (same as the queue index within the AOS channel) of the
    // message that we are currently sending or, if we are between messages,
    // the next message we will send.
    uint32_t current_queue_index = 0;
    // Index of the next packet to send within current_queue_index (large
    // messages are broken into multiple packets, as we have encountered
    // issues with how some WebRTC implementations handle large packets).
    size_t next_packet_number = 0;
    // The last queue/packet index reported by the client.
    uint32_t reported_queue_index = 0;
    size_t reported_packet_index = 0;
  };
  struct Message {
    uint32_t index = 0xffffffff;
    std::vector<std::shared_ptr<struct mbuf>> data;
  };

  std::shared_ptr<struct mbuf> NextBuffer(ChannelInformation *channel);
  void SkipToLastMessage(ChannelInformation *channel);

  std::unique_ptr<RawFetcher> fetcher_;
  int channel_index_;
  // If set, will always build up a buffer of the most recent buffer_size_
  // messages. If store_history_ is *not* set we will only buffer up messages
  // while there is an active listener.
  bool store_history_;
  int buffer_size_;
  std::deque<Message> message_buffer_;
  // The ScopedDataChannel that we use for actually sending data over WebRTC
  // is stored using a weak_ptr because:
  // (a) There are some dangers of accidentally creating circular dependencies
  //     that prevent a ScopedDataChannel from ever being destroyed.
  // (b) The inter-dependencies involved are complicated enough that we want
  //     to be able to check whether someone has destroyed the ScopedDataChannel
  //     before using it (if it has been destroyed and the Subscriber still
  //     wants to use it, that is a bug, but checking for bugs is useful).
  // This particular location *may* be able to get away with a shared_ptr, but
  // because the ScopedDataChannel effectively destroys itself (see
  // ScopedDataChannel::StaticDataChannelCloseHandler) while also potentially
  // holding references to other objects (e.g., through the various handlers
  // that can be registered), creating unnecessary shared_ptr's is dubious.
  std::vector<std::pair<std::weak_ptr<ScopedDataChannel>, ChannelInformation>>
      channels_;
};

// Class to manage a WebRTC connection to a browser.
class ApplicationConnection {
 public:
  ApplicationConnection(
      ::seasocks::Server *server, ::seasocks::WebSocket *sock,
      const std::vector<std::unique_ptr<Subscriber>> &subscribers,
      const aos::FlatbufferDetachedBuffer<aos::Configuration> &config,
      const EventLoop *event_loop);

  ~ApplicationConnection();

  // Handles a SDP sent through the negotiation channel.
  void OnSdp(const char *sdp);
  // Handles a ICE candidate sent through the negotiation channel.
  void OnIce(const WebSocketIce *ice);

 private:
  void LocalCandidate(
      struct rawrtc_peer_connection_ice_candidate *const candidate,
      char const *const url);

  // Handles a signaling channel being made.
  void OnDataChannel(std::shared_ptr<ScopedDataChannel> channel);

  // Handles data coming in on the signaling channel requesting subscription.
  void HandleSignallingData(
      struct mbuf *const
          buffer,  // nullable (in case partial delivery has been requested)
      const enum rawrtc_data_channel_message_flag /*flags*/);

  RawRTCConnection connection_;

  ::seasocks::Server *server_;
  ::seasocks::WebSocket *sock_;

  struct ChannelState {
    std::shared_ptr<ScopedDataChannel> data_channel;
    bool requested = true;
  };

  std::map<int, ChannelState> channels_;
  const std::vector<std::unique_ptr<Subscriber>> &subscribers_;

  const std::vector<FlatbufferDetachedBuffer<MessageHeader>> config_headers_;

  const EventLoop *const event_loop_;

  std::shared_ptr<ScopedDataChannel> channel_;
};

}  // namespace web_proxy
}  // namespace aos

#endif  // AOS_NETWORK_WEB_PROXY_H_
