#ifndef AOS_UTIL_FOXGLOVE_WEBSOCKET_LIB_H_
#define AOS_UTIL_FOXGLOVE_WEBSOCKET_LIB_H_
#include <map>
#include <set>

#include "aos/events/event_loop.h"
#include "foxglove/websocket/server.hpp"

namespace aos {
// This class implements a live AOS -> Foxglove Websocket Protocol connection,
// making use of the implementation at
// https://github.com/foxglove/ws-protocol/tree/main/cpp to send JSON messages
// to a foxglove studio client.
// See foxglove_websocket.cc for some usage notes.
class FoxgloveWebsocketServer {
 public:
  // Whether to serialize the messages into the MCAP file as JSON or
  // flatbuffers.
  enum class Serialization {
    kJson,
    kFlatbuffer,
  };
  enum class FetchPinnedChannels {
    kYes,
    kNo,
  };
  FoxgloveWebsocketServer(aos::EventLoop *event_loop, uint32_t port,
                          Serialization serialization,
                          FetchPinnedChannels fetch_pinned_channels);
  ~FoxgloveWebsocketServer();

 private:
  typedef foxglove::websocket::ChannelId ChannelId;

  struct FetcherState {
    std::unique_ptr<aos::RawFetcher> fetcher;
    // Whether the current message in the fetcher has been sent to the client.
    // Starts as true because the fetcher starts with no data.
    // This is necessary because we have to send all of our messages out
    // in order, which we can only do once we know the timestamp of each
    // message. And we can only know the timestamp after having called Fetch()
    // on the fetcher. Once we get around to actually sending the data, we can
    // set this to true so that we know it is safe to call FetchNext() again.
    bool sent_current_message = true;
  };

  aos::EventLoop *event_loop_;
  const Serialization serialization_;
  const FetchPinnedChannels fetch_pinned_channels_;
  foxglove::websocket::Server server_;
  // A map of fetchers for every single channel that could be subscribed to.
  std::map<ChannelId, FetcherState> fetchers_;
  // The set of channels that we have clients actively subscribed to.
  std::set<ChannelId> active_channels_;
};
}  // namespace aos
#endif  // AOS_UTIL_FOXGLOVE_WEBSOCKET_LIB_H_
