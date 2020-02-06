#include "aos/network/web_proxy.h"
#include "aos/network/web_proxy_generated.h"
#include "api/create_peerconnection_factory.h"
#include "glog/logging.h"

namespace aos {
namespace web_proxy {

namespace {
// Based on webrtc examples. In our controlled environment we expect setting sdp
// to always succeed, and we can't do anything about a failure, so just ignore
// everything.
class DummySetSessionDescriptionObserver
    : public webrtc::SetSessionDescriptionObserver {
 public:
  static DummySetSessionDescriptionObserver *Create() {
    return new rtc::RefCountedObject<DummySetSessionDescriptionObserver>();
  }
  virtual void OnSuccess() {}
  virtual void OnFailure(webrtc::RTCError error) {}
};

}  // namespace

WebsocketHandler::WebsocketHandler(::seasocks::Server *server)
    : server_(server) {}

void WebsocketHandler::onConnect(::seasocks::WebSocket *sock) {
  std::unique_ptr<Connection> conn =
      std::make_unique<Connection>(sock, server_);
  connections_.insert({sock, std::move(conn)});
}

void WebsocketHandler::onData(::seasocks::WebSocket *sock, const uint8_t *data,
                              size_t size) {
  connections_[sock]->HandleWebSocketData(data, size);
}

void WebsocketHandler::onDisconnect(::seasocks::WebSocket *sock) {
  connections_.erase(sock);
}

Connection::Connection(::seasocks::WebSocket *sock, ::seasocks::Server *server)
    : sock_(sock), server_(server) {}

// Function called for web socket data. Parses the flatbuffer and handles it
// appropriately.
void Connection::HandleWebSocketData(const uint8_t *data, size_t size) {
  const WebSocketMessage *message =
      flatbuffers::GetRoot<WebSocketMessage>(data);
  switch (message->payload_type()) {
    case Payload::WebSocketSdp: {
      const WebSocketSdp *offer = message->payload_as_WebSocketSdp();
      if (offer->type() != SdpType::OFFER) {
        LOG(WARNING) << "Got the wrong sdp type from client";
        break;
      }
      const flatbuffers::String *sdp = offer->payload();
      webrtc::SdpParseError error;
      std::unique_ptr<webrtc::SessionDescriptionInterface> desc =
          CreateSessionDescription(webrtc::SdpType::kOffer, sdp->str(), &error);
      if (!desc) {
        LOG(WARNING) << "Failed to parse sdp description: "
                     << error.description;
        // TODO(alex): send a message back to browser for failure.
        break;
      }

      // We can only start creating the PeerConnection once we have something to
      // give it, so we wait until we get an offer before starting.
      webrtc::PeerConnectionInterface::RTCConfiguration config;
      config.sdp_semantics = webrtc::SdpSemantics::kUnifiedPlan;
      config.enable_dtls_srtp = true;

      std::unique_ptr<rtc::Thread> signaling_thread = rtc::Thread::Create();
      signaling_thread->SetName("signaling_thread", nullptr);
      signaling_thread->Start();

      webrtc::PeerConnectionFactoryDependencies factory_deps;
      factory_deps.signaling_thread = signaling_thread.release();
      rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> factory =
          CreateModularPeerConnectionFactory(std::move(factory_deps));

      peer_connection_ =
          factory->CreatePeerConnection(config, nullptr, nullptr, this);

      peer_connection_->SetRemoteDescription(
          DummySetSessionDescriptionObserver::Create(), desc.release());

      peer_connection_->CreateAnswer(
          this, webrtc::PeerConnectionInterface::RTCOfferAnswerOptions());
      break;
    }
    case Payload::WebSocketIce: {
      const WebSocketIce *ice = message->payload_as_WebSocketIce();
      std::string candidate = ice->candidate()->str();
      std::string sdpMid = ice->sdpMid()->str();
      int sdpMLineIndex = ice->sdpMLineIndex();
      webrtc::SdpParseError error;
      webrtc::IceCandidateInterface *ice_candidate =
          webrtc::CreateIceCandidate(sdpMid, sdpMLineIndex, candidate, &error);
      if (!ice_candidate) {
        LOG(WARNING) << "Failed to parse ice candidate: " << error.description;
        // TODO(alex): send a message back to browser for failure.
        break;
      }
      peer_connection_->AddIceCandidate(ice_candidate);
      break;
    }
    default: { break; }
  }
}

void Connection::OnDataChannel(
    rtc::scoped_refptr<webrtc::DataChannelInterface> channel) {
  data_channel_ = channel;
  data_channel_->RegisterObserver(this);
}

void Connection::OnIceCandidate(
    const webrtc::IceCandidateInterface *candidate) {
  flatbuffers::FlatBufferBuilder fbb(512);
  std::string ice_string;
  candidate->ToString(&ice_string);

  flatbuffers::Offset<WebSocketIce> ice_fb = CreateWebSocketIceDirect(
      fbb, ice_string.c_str(), candidate->sdp_mid().c_str(),
      candidate->sdp_mline_index());
  flatbuffers::Offset<WebSocketMessage> ice_message =
      CreateWebSocketMessage(fbb, Payload::WebSocketIce, ice_fb.Union());
  fbb.Finish(ice_message);

  server_->execute(std::make_shared<UpdateData>(sock_, fbb.Release()));
}

// This is the callback for creating an sdp. We have to manually assign it
// locally and send it to the client.
void Connection::OnSuccess(webrtc::SessionDescriptionInterface *desc) {
  peer_connection_->SetLocalDescription(
      DummySetSessionDescriptionObserver::Create(), desc);
  flatbuffers::FlatBufferBuilder fbb(512);
  std::string answer_string;
  desc->ToString(&answer_string);
  flatbuffers::Offset<WebSocketSdp> sdp_fb =
      CreateWebSocketSdpDirect(fbb, SdpType::ANSWER, answer_string.c_str());
  flatbuffers::Offset<WebSocketMessage> answer_message =
      CreateWebSocketMessage(fbb, Payload::WebSocketSdp, sdp_fb.Union());
  fbb.Finish(answer_message);

  server_->execute(std::make_shared<UpdateData>(sock_, fbb.Release()));
}

// Receive and respond to a DataChannel message. Temporarily acting as a
// "PONG", but will change to handle "Connect" subscription messages.
void Connection::OnMessage(const webrtc::DataBuffer &buffer) {
  // This is technically disallowed by webrtc, But doesn't seem to cause major
  // problems. At least for the small data tested manually. Send should be
  // called from outside this call stack.
  data_channel_->Send(buffer);
}

}  // namespace web_proxy
}  // namespace aos
