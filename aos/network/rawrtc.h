#ifndef AOS_NETWORK_RAWRTC_H_
#define AOS_NETWORK_RAWRTC_H_

#include <functional>
#include <string>

extern "C" {
#include <rawrtc.h>

#include "external/com_github_rawrtc_rawrtc_common/include/rawrtcc/utils.h"
}

#include "flatbuffers/flatbuffers.h"
#include "glog/logging.h"

namespace aos {
namespace web_proxy {

// TODO(austin): This doesn't allow streaming data in.
#define CHECK_RAWRTC(x)                            \
  [&]() {                                          \
    enum rawrtc_code r = x;                        \
    return CHECK(r == RAWRTC_CODE_SUCCESS)         \
           << " actual " << rawrtc_code_to_str(r); \
  }()

#define CHECK_RAWRTC_IGNORE(x, i)           \
  [&]() {                                   \
    enum rawrtc_code r = x;                 \
    for (auto w : i) {                      \
      if (w == r) return;                   \
    }                                       \
    return CHECK(r == RAWRTC_CODE_SUCCESS); \
  }()

// Wrapper around a RawRTC data channel to manage it's lifetime and provide C++
// callbacks for all the callbacks.
//
// There are 3 phases of the object's lifetime.
// 1) Initialization.  Callbacks can be set here.
// 2) Open. Calling Open transitions the channel to be open and triggers the
//    on_open callback to be called.
// 3) Close.  This must be called before destroying the channel and calls the
//    on_close callback and shuts down the channel.
class ScopedDataChannel {
 public:
  ScopedDataChannel();
  ScopedDataChannel(const ScopedDataChannel &) = delete;
  ScopedDataChannel &operator=(const ScopedDataChannel &) = delete;

  ~ScopedDataChannel();

  // Setters for all the callbacks.  These may be called whenever.

  // Registers a callback to be called when the channel is opened.  This only
  // gets called once during or after Open is called.
  void set_on_open(std::function<void()> &&fn) { on_open_ = std::move(fn); }

  // Registers a callback to be called when the channel is closed.  This only
  // gets called once during or after Close is called.
  void set_on_close(std::function<void()> &&fn) { on_close_ = std::move(fn); }

  void set_on_buffered_amount_low(std::function<void()> &&fn) {
    on_buffered_amount_low_ = std::move(fn);
  }
  void set_on_error(std::function<void()> &&fn) { on_error_ = std::move(fn); }
  void set_on_message(
      std::function<void(struct mbuf *const,
                         enum rawrtc_data_channel_message_flag const)> &&fn) {
    on_message_ = std::move(fn);
  }

  // Opens the channel on the provided connection with the provided label.  This
  // is separate so we can optionally register callbacks before opening.
  void Open(struct rawrtc_peer_connection *connection,
            const std::string &label);
  // Takes over an already open channel.
  void Open(struct rawrtc_data_channel *channel);

  // Closes the channel.  It must be open first.
  void Close();

  // Sends a buffer.
  void Send(const ::flatbuffers::DetachedBuffer &buffer);
  void Send(struct mbuf *buffer);

  std::string_view label() const { return label_; }

  // Returns the amount of data buffered.
  uint64_t buffered_amount();

 private:
  // Trampolines from C -> C++.
  static void StaticDataChannelOpenHandler(void *const arg);
  static void StaticBufferedAmountLowHandler(void *const arg);
  static void StaticDataChannelErrorHandler(void *const arg);
  static void StaticDataChannelCloseHandler(void *const arg);
  static void StaticDataChannelMessageHandler(
      struct mbuf *const
          buffer,  // nullable (in case partial delivery has been requested)
      enum rawrtc_data_channel_message_flag const flags, void *const arg);

  // Our channel and the label for it.
  std::string label_;
  struct rawrtc_data_channel *data_channel_ = nullptr;

  bool opened_ = false;
  bool closed_ = false;

  std::function<void()> on_open_;
  std::function<void()> on_buffered_amount_low_;
  std::function<void()> on_error_;
  std::function<void()> on_close_;
  std::function<void(struct mbuf *const,
                     enum rawrtc_data_channel_message_flag const)>
      on_message_;

  // Self referential pointer to keep ourselves in scope until close() gets
  // called.
  std::shared_ptr<ScopedDataChannel> self_;
};

// Wraper around a RawRTC connection to both manage it's lifetime and provide
// std::function interfaces for the callbacks.
class RawRTCConnection {
 public:
  RawRTCConnection();

  virtual ~RawRTCConnection();

  void set_on_negotiation_needed(std::function<void()> &&fn) {
    on_negotiation_needed_ = std::move(fn);
  }
  void set_on_local_candidate(
      std::function<void(struct rawrtc_peer_connection_ice_candidate *,
                         char const *)> &&fn) {
    on_local_candidate_ = std::move(fn);
  }
  // Sets the handler for a peer connection local candidate error.  Arguments
  // are the candidate, URL, error_code and error_text.
  void set_on_peer_connection_local_candidate_error(
      std::function<void(struct rawrtc_peer_connection_ice_candidate *,
                         char const *, uint16_t, char const *)> &&fn) {
    on_peer_connection_local_candidate_error_ = std::move(fn);
  }
  void set_on_signaling_state_change(
      std::function<void(enum rawrtc_signaling_state const)> &&fn) {
    on_signaling_state_change_ = std::move(fn);
  }
  void set_on_ice_transport_state_change(
      std::function<void(const enum rawrtc_ice_transport_state)> &&fn) {
    on_ice_transport_state_change_ = std::move(fn);
  }
  void set_on_ice_gatherer_state_change(
      std::function<void(const enum rawrtc_ice_gatherer_state)> &&fn) {
    on_ice_gatherer_state_change_ = std::move(fn);
  }
  void set_on_connection_state_change(
      std::function<void(const enum rawrtc_peer_connection_state)> &&fn) {
    on_connection_state_change_ = std::move(fn);
  }

  // TODO(austin): Really, this should be a ScopedDataChannel object.
  void set_on_data_channel(
      std::function<void(std::shared_ptr<ScopedDataChannel>)> &&fn) {
    on_data_channel_ = std::move(fn);
  }

  // Opens the connection.  This lets us register callbacks before starting it.
  void Open();

  // Returns the connection if Open has been called.
  struct rawrtc_peer_connection *connection() {
    return connection_;
  }

 private:
  // Trampolines from C -> C++.
  static void StaticNegotiationNeededHandler(void *const arg);
  static void StaticLocalCandidateHandler(
      struct rawrtc_peer_connection_ice_candidate *const candidate,
      char const *const url, void *const arg);
  static void StaticPeerConnectionLocalCandidateErrorHandler(
      struct rawrtc_peer_connection_ice_candidate *const candidate,
      char const *const url, uint16_t const error_code,
      char const *const error_text, void *const arg);
  static void StaticSignalingStateChangeHandler(
      const enum rawrtc_signaling_state state, void *const arg);
  static void StaticIceTransportStateChangeHandler(
      const enum rawrtc_ice_transport_state state, void *const arg);
  static void StaticIceGathererStateChangeHandler(
      const enum rawrtc_ice_gatherer_state state, void *const arg);
  static void StaticConnectionStateChangeHandler(
      const enum rawrtc_peer_connection_state state, void *const arg);
  static void StaticDataChannelHandler(
      struct rawrtc_data_channel *const channel, void *const arg);

  // The connection.
  struct rawrtc_peer_connection *connection_ = nullptr;

  // Callbacks
  std::function<void()> on_negotiation_needed_;
  std::function<void(struct rawrtc_peer_connection_ice_candidate *,
                     char const *)>
      on_local_candidate_;
  std::function<void(struct rawrtc_peer_connection_ice_candidate *,
                     char const *, uint16_t, char const *)>
      on_peer_connection_local_candidate_error_;
  std::function<void(enum rawrtc_signaling_state const)>
      on_signaling_state_change_;
  std::function<void(const enum rawrtc_ice_transport_state)>
      on_ice_transport_state_change_;
  std::function<void(const enum rawrtc_ice_gatherer_state)>
      on_ice_gatherer_state_change_;
  std::function<void(const enum rawrtc_peer_connection_state)>
      on_connection_state_change_;
  std::function<void(std::shared_ptr<ScopedDataChannel>)> on_data_channel_;
};

}  // namespace web_proxy
}  // namespace aos

#endif  // AOS_NETWORK_RAWRTC_H_
