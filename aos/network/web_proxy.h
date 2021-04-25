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

// Basic class that handles receiving new websocket connections. Creates a new
// Connection to manage the rest of the negotiation and data passing. When the
// websocket closes, it deletes the Connection.
class WebsocketHandler : public ::seasocks::WebSocket::Handler {
 public:
  WebsocketHandler(::seasocks::Server *server, aos::EventLoop *event_loop,
                   int buffer_size);
  void onConnect(::seasocks::WebSocket *sock) override;
  void onData(::seasocks::WebSocket *sock, const uint8_t *data,
              size_t size) override;
  void onDisconnect(::seasocks::WebSocket *sock) override;

 private:
  ::seasocks::Server *server_;
  std::vector<std::unique_ptr<Subscriber>> subscribers_;
  const aos::FlatbufferDetachedBuffer<aos::Configuration> config_;

  std::map<::seasocks::WebSocket *, std::unique_ptr<ApplicationConnection>>
      connections_;

  EventLoop *const event_loop_;
};

// Wrapper class that manages the seasocks server and WebsocketHandler.
class WebProxy {
 public:
  WebProxy(aos::EventLoop *event_loop, int buffer_size);
  WebProxy(aos::ShmEventLoop *event_loop, int buffer_size);
  ~WebProxy();

  void SetDataPath(const char *path) { server_.setStaticPath(path); }

 private:
  WebProxy(aos::EventLoop *event_loop, aos::internal::EPoll *epoll,
           int buffer_size);

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
             int buffer_size)
      : fetcher_(std::move(fetcher)),
        channel_index_(channel_index),
        buffer_size_(buffer_size) {}

  void RunIteration();

  void AddListener(std::shared_ptr<ScopedDataChannel> data_channel,
                   TransferMethod transfer_method);

  void RemoveListener(std::shared_ptr<ScopedDataChannel> data_channel);

 private:
  struct ChannelInformation {
    TransferMethod transfer_method;
    uint32_t current_queue_index = 0;
    size_t next_packet_number = 0;
  };
  struct Message {
    uint32_t index = 0xffffffff;
    std::vector<std::shared_ptr<struct mbuf>> data;
  };

  std::shared_ptr<struct mbuf> NextBuffer(ChannelInformation *channel);
  void SkipToLastMessage(ChannelInformation *channel);

  std::unique_ptr<RawFetcher> fetcher_;
  int channel_index_;
  int buffer_size_;
  std::deque<Message> message_buffer_;
  std::map<std::shared_ptr<ScopedDataChannel>, ChannelInformation> channels_;
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
