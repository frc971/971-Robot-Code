#define GST_USE_UNSTABLE_API
#define GST_DISABLE_REGISTRY 1

#include <glib-unix.h>
#include <glib.h>
#include <gst/app/app.h>
#include <gst/gst.h>
#include <gst/sdp/sdp.h>
#include <gst/webrtc/icetransport.h>
#include <gst/webrtc/webrtc.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <map>
#include <thread>

#include "absl/strings/str_format.h"
#include "aos/events/glib_main_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/web_proxy_generated.h"
#include "aos/seasocks/seasocks_logger.h"
#include "flatbuffers/flatbuffers.h"
#include "frc971/vision/vision_generated.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "internal/Embedded.h"
#include "seasocks/Server.h"
#include "seasocks/StringUtil.h"
#include "seasocks/WebSocket.h"

extern "C" {
GST_PLUGIN_STATIC_DECLARE(app);
GST_PLUGIN_STATIC_DECLARE(coreelements);
GST_PLUGIN_STATIC_DECLARE(dtls);
GST_PLUGIN_STATIC_DECLARE(nice);
GST_PLUGIN_STATIC_DECLARE(rtp);
GST_PLUGIN_STATIC_DECLARE(rtpmanager);
GST_PLUGIN_STATIC_DECLARE(srtp);
GST_PLUGIN_STATIC_DECLARE(webrtc);
GST_PLUGIN_STATIC_DECLARE(video4linux2);
GST_PLUGIN_STATIC_DECLARE(videoconvert);
GST_PLUGIN_STATIC_DECLARE(videoparsersbad);
GST_PLUGIN_STATIC_DECLARE(videorate);
GST_PLUGIN_STATIC_DECLARE(videoscale);
GST_PLUGIN_STATIC_DECLARE(videotestsrc);
GST_PLUGIN_STATIC_DECLARE(x264);
}

DEFINE_string(config, "y2022/aos_config.json",
              "Name of the config file to replay using.");
DEFINE_string(device, "/dev/video0",
              "Camera fd. Ignored if reading from channel");
DEFINE_string(data_dir, "image_streamer_www",
              "Directory to serve data files from");
DEFINE_int32(width, 400, "Image width");
DEFINE_int32(height, 300, "Image height");
DEFINE_int32(framerate, 25, "Framerate (FPS)");
DEFINE_int32(brightness, 50, "Camera brightness");
DEFINE_int32(exposure, 300, "Manual exposure");
DEFINE_int32(bitrate, 500000, "H264 encode bitrate");
DEFINE_int32(min_port, 5800, "Min rtp port");
DEFINE_int32(max_port, 5810, "Max rtp port");
DEFINE_string(listen_on, "",
              "Channel on which to receive frames from. Used in place of "
              "internal V4L2 reader. Note: width and height MUST match the "
              "expected size of channel images.");

class Connection;

using aos::web_proxy::Payload;
using aos::web_proxy::SdpType;
using aos::web_proxy::WebSocketIce;
using aos::web_proxy::WebSocketMessage;
using aos::web_proxy::WebSocketSdp;

class GstSampleSource {
 public:
  GstSampleSource() = default;

  virtual ~GstSampleSource() = default;

 private:
  GstSampleSource(const GstSampleSource &) = delete;
};

class V4L2Source : public GstSampleSource {
 public:
  V4L2Source(std::function<void(GstSample *)> callback)
      : callback_(std::move(callback)) {
    GError *error = NULL;

    // Create pipeline to read from camera, pack into rtp stream, and dump
    // stream to callback. v4l2 device should already be configured with correct
    // bitrate from v4l2-ctl. do-timestamp marks the time the frame was taken to
    // track when it should be dropped under latency.

    // With the Pi's hardware encoder, we can encode and package the stream once
    // and the clients will jump in at any point unsynchronized. With the stream
    // from x264enc this doesn't seem to work. For now, just reencode for each
    // client since we don't expect more than 1 or 2.

    pipeline_ = gst_parse_launch(
        absl::StrFormat("v4l2src device=%s do-timestamp=true "
                        "extra-controls=\"c,brightness=%d,auto_exposure=1,"
                        "exposure_time_absolute=%d\" ! "
                        "video/x-raw,width=%d,height=%d,framerate=%d/"
                        "1,format=YUY2 ! appsink "
                        "name=appsink "
                        "emit-signals=true sync=false async=false "
                        "caps=video/x-raw,format=YUY2",
                        FLAGS_device, FLAGS_brightness, FLAGS_exposure,
                        FLAGS_width, FLAGS_height, FLAGS_framerate)
            .c_str(),
        &error);

    if (error != NULL) {
      LOG(FATAL) << "Could not create v4l2 pipeline: " << error->message;
    }

    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink");
    if (appsink_ == NULL) {
      LOG(FATAL) << "Could not get appsink";
    }

    g_signal_connect(appsink_, "new-sample",
                     G_CALLBACK(V4L2Source::OnSampleCallback),
                     static_cast<gpointer>(this));

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  }

  ~V4L2Source() {
    if (pipeline_ != NULL) {
      gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_NULL);
      gst_object_unref(GST_OBJECT(pipeline_));
      gst_object_unref(GST_OBJECT(appsink_));
    }
  }

 private:
  static GstFlowReturn OnSampleCallback(GstElement *, gpointer user_data) {
    static_cast<V4L2Source *>(user_data)->OnSample();
    return GST_FLOW_OK;
  }

  void OnSample() {
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink_));
    if (sample == NULL) {
      LOG(WARNING) << "Received null sample";
      return;
    }
    callback_(sample);
    gst_sample_unref(sample);
  }

  GstElement *pipeline_;
  GstElement *appsink_;

  std::function<void(GstSample *)> callback_;
};

class ChannelSource : public GstSampleSource {
 public:
  ChannelSource(aos::ShmEventLoop *event_loop,
                std::function<void(GstSample *)> callback)
      : callback_(std::move(callback)) {
    event_loop->MakeWatcher(
        FLAGS_listen_on,
        [this](const frc971::vision::CameraImage &image) { OnImage(image); });
  }

 private:
  void OnImage(const frc971::vision::CameraImage &image) {
    if (!image.has_rows() || !image.has_cols() || !image.has_data()) {
      VLOG(2) << "Skipping CameraImage with no data";
      return;
    }
    CHECK_EQ(image.rows(), FLAGS_height);
    CHECK_EQ(image.cols(), FLAGS_width);

    GBytes *bytes = g_bytes_new(image.data()->data(), image.data()->size());
    GstBuffer *buffer = gst_buffer_new_wrapped_bytes(bytes);

    GST_BUFFER_PTS(buffer) = image.monotonic_timestamp_ns();

    GstCaps *caps = CHECK_NOTNULL(gst_caps_new_simple(
        "video/x-raw", "width", G_TYPE_INT, image.cols(), "height", G_TYPE_INT,
        image.rows(), "format", G_TYPE_STRING, "YUY2", nullptr));

    GstSample *sample = gst_sample_new(buffer, caps, nullptr, nullptr);

    callback_(sample);

    gst_sample_unref(sample);
    gst_caps_unref(caps);
    gst_buffer_unref(buffer);
    g_bytes_unref(bytes);
  }

  std::function<void(GstSample *)> callback_;
};

// Basic class that handles receiving new websocket connections. Creates a new
// Connection to manage the rest of the negotiation and data passing. When the
// websocket closes, it deletes the Connection.
class WebsocketHandler : public ::seasocks::WebSocket::Handler {
 public:
  WebsocketHandler(aos::ShmEventLoop *event_loop, ::seasocks::Server *server);
  ~WebsocketHandler() override = default;

  void onConnect(::seasocks::WebSocket *sock) override;
  void onData(::seasocks::WebSocket *sock, const uint8_t *data,
              size_t size) override;
  void onDisconnect(::seasocks::WebSocket *sock) override;

 private:
  void OnSample(GstSample *sample);

  std::map<::seasocks::WebSocket *, std::unique_ptr<Connection>> connections_;
  ::seasocks::Server *server_;
  std::unique_ptr<GstSampleSource> source_;

  aos::Sender<frc971::vision::CameraImage> sender_;
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

class Connection {
 public:
  Connection(::seasocks::WebSocket *sock, ::seasocks::Server *server);

  ~Connection();

  void HandleWebSocketData(const uint8_t *data, size_t size);

  void OnSample(GstSample *sample);

 private:
  static void OnOfferCreatedCallback(GstPromise *promise, gpointer user_data) {
    static_cast<Connection *>(user_data)->OnOfferCreated(promise);
  }

  static void OnNegotiationNeededCallback(GstElement *, gpointer user_data) {
    static_cast<Connection *>(user_data)->OnNegotiationNeeded();
  }

  static void OnIceCandidateCallback(GstElement *, guint mline_index,
                                     gchar *candidate, gpointer user_data) {
    static_cast<Connection *>(user_data)->OnIceCandidate(mline_index,
                                                         candidate);
  }

  void OnOfferCreated(GstPromise *promise);
  void OnNegotiationNeeded();
  void OnIceCandidate(guint mline_index, gchar *candidate);

  ::seasocks::WebSocket *sock_;
  ::seasocks::Server *server_;

  GstElement *pipeline_;
  GstElement *webrtcbin_;
  GstElement *appsrc_;

  bool first_sample_ = true;
};

WebsocketHandler::WebsocketHandler(aos::ShmEventLoop *event_loop,
                                   ::seasocks::Server *server)
    : server_(server) {
  if (FLAGS_listen_on.empty()) {
    sender_ = event_loop->MakeSender<frc971::vision::CameraImage>("/camera");
    source_ =
        std::make_unique<V4L2Source>([this](auto sample) { OnSample(sample); });
  } else {
    source_ = std::make_unique<ChannelSource>(
        event_loop, [this](auto sample) { OnSample(sample); });
  }
}

void WebsocketHandler::onConnect(::seasocks::WebSocket *sock) {
  std::unique_ptr<Connection> conn =
      std::make_unique<Connection>(sock, server_);
  connections_.insert({sock, std::move(conn)});
}

void WebsocketHandler::onData(::seasocks::WebSocket *sock, const uint8_t *data,
                              size_t size) {
  connections_[sock]->HandleWebSocketData(data, size);
}

void WebsocketHandler::OnSample(GstSample *sample) {
  for (auto iter = connections_.begin(); iter != connections_.end(); ++iter) {
    iter->second->OnSample(sample);
  }

  if (sender_.valid()) {
    const GstCaps *caps = CHECK_NOTNULL(gst_sample_get_caps(sample));
    CHECK_GT(gst_caps_get_size(caps), 0U);
    const GstStructure *str = gst_caps_get_structure(caps, 0);

    gint width;
    gint height;

    CHECK(gst_structure_get_int(str, "width", &width));
    CHECK(gst_structure_get_int(str, "height", &height));

    GstBuffer *buffer = CHECK_NOTNULL(gst_sample_get_buffer(sample));

    const gsize size = gst_buffer_get_size(buffer);

    auto builder = sender_.MakeBuilder();

    uint8_t *image_data;
    auto image_offset =
        builder.fbb()->CreateUninitializedVector(size, &image_data);
    gst_buffer_extract(buffer, 0, image_data, size);

    auto image_builder = builder.MakeBuilder<frc971::vision::CameraImage>();
    image_builder.add_rows(height);
    image_builder.add_cols(width);
    image_builder.add_data(image_offset);

    builder.CheckOk(builder.Send(image_builder.Finish()));
  }
}

void WebsocketHandler::onDisconnect(::seasocks::WebSocket *sock) {
  connections_.erase(sock);
}

Connection::Connection(::seasocks::WebSocket *sock, ::seasocks::Server *server)
    : sock_(sock), server_(server) {
  GError *error = NULL;

  // Build pipeline to read data from application into pipeline, place in
  // webrtcbin group, and stream.

  pipeline_ = gst_parse_launch(
      // aggregate-mode should be zero-latency but this drops the stream on
      // bitrate spikes for some reason - probably the weak CPU on the pi.
      absl::StrFormat(
          "webrtcbin name=webrtcbin appsrc "
          "name=appsrc block=false "
          "is-live=true "
          "format=3 max-buffers=0 leaky-type=2 "
          "caps=video/x-raw,width=%d,height=%d,format=YUY2 ! videoconvert ! "
          "x264enc bitrate=%d speed-preset=ultrafast "
          "tune=zerolatency key-int-max=15 sliced-threads=true ! "
          "video/x-h264,profile=constrained-baseline ! h264parse ! "
          "rtph264pay "
          "config-interval=-1 name=payloader aggregate-mode=none ! "
          "application/"
          "x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000 !"
          "webrtcbin. ",
          FLAGS_width, FLAGS_height, FLAGS_bitrate / 1000)
          .c_str(),
      &error);

  if (error != NULL) {
    LOG(FATAL) << "Could not create WebRTC pipeline: " << error->message;
  }

  webrtcbin_ = gst_bin_get_by_name(GST_BIN(pipeline_), "webrtcbin");
  if (webrtcbin_ == NULL) {
    LOG(FATAL) << "Could not initialize webrtcbin";
  }

  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsrc");
  if (appsrc_ == NULL) {
    LOG(FATAL) << "Could not initialize appsrc";
  }

  {
    GArray *transceivers;
    g_signal_emit_by_name(webrtcbin_, "get-transceivers", &transceivers);
    if (transceivers == NULL || transceivers->len <= 0) {
      LOG(FATAL) << "Could not initialize transceivers";
    }

    GstWebRTCRTPTransceiver *trans =
        g_array_index(transceivers, GstWebRTCRTPTransceiver *, 0);
    g_object_set(trans, "direction",
                 GST_WEBRTC_RTP_TRANSCEIVER_DIRECTION_SENDONLY, nullptr);

    g_array_unref(transceivers);
  }

  {
    GstObject *ice = nullptr;
    g_object_get(G_OBJECT(webrtcbin_), "ice-agent", &ice, nullptr);
    CHECK_NOTNULL(ice);

    g_object_set(ice, "min-rtp-port", FLAGS_min_port, "max-rtp-port",
                 FLAGS_max_port, nullptr);

    // We don't need upnp on a local network.
    {
      GstObject *nice = nullptr;
      g_object_get(ice, "agent", &nice, nullptr);
      CHECK_NOTNULL(nice);

      g_object_set(nice, "upnp", false, nullptr);
      g_object_unref(nice);
    }

    gst_object_unref(ice);
  }

  g_signal_connect(webrtcbin_, "on-negotiation-needed",
                   G_CALLBACK(Connection::OnNegotiationNeededCallback),
                   static_cast<gpointer>(this));

  g_signal_connect(webrtcbin_, "on-ice-candidate",
                   G_CALLBACK(Connection::OnIceCandidateCallback),
                   static_cast<gpointer>(this));

  gst_element_set_state(pipeline_, GST_STATE_READY);
  gst_element_set_state(pipeline_, GST_STATE_PLAYING);
}

Connection::~Connection() {
  if (pipeline_ != NULL) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);

    gst_object_unref(GST_OBJECT(webrtcbin_));
    gst_object_unref(GST_OBJECT(pipeline_));
    gst_object_unref(GST_OBJECT(appsrc_));
  }
}

void Connection::OnSample(GstSample *sample) {
  GstFlowReturn response =
      gst_app_src_push_sample(GST_APP_SRC(appsrc_), sample);
  if (response != GST_FLOW_OK) {
    LOG(WARNING) << "Sample pushed, did not receive OK";
  }

  // Since the stream is already running (the camera turns on with
  // image_streamer) we need to tell the new appsrc where
  // we are starting in the stream so it can catch up immediately.
  if (first_sample_) {
    GstPad *src = gst_element_get_static_pad(appsrc_, "src");
    if (src == NULL) {
      return;
    }

    GstSegment *segment = gst_sample_get_segment(sample);
    GstBuffer *buffer = gst_sample_get_buffer(sample);

    guint64 offset = gst_segment_to_running_time(segment, GST_FORMAT_TIME,
                                                 GST_BUFFER_PTS(buffer));
    LOG(INFO) << "Fixing offset " << offset;
    gst_pad_set_offset(src, -offset);

    gst_object_unref(GST_OBJECT(src));
    first_sample_ = false;
  }
}

void Connection::OnOfferCreated(GstPromise *promise) {
  LOG(INFO) << "OnOfferCreated";

  GstWebRTCSessionDescription *offer = NULL;
  gst_structure_get(gst_promise_get_reply(promise), "offer",
                    GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, NULL);
  gst_promise_unref(promise);

  {
    std::unique_ptr<GstPromise, decltype(&gst_promise_unref)>
        local_desc_promise(gst_promise_new(), &gst_promise_unref);
    g_signal_emit_by_name(webrtcbin_, "set-local-description", offer,
                          local_desc_promise.get());
    gst_promise_interrupt(local_desc_promise.get());
  }

  GstSDPMessage *sdp_msg = offer->sdp;
  std::string sdp_str(gst_sdp_message_as_text(sdp_msg));

  LOG(INFO) << "Negotiation offer created:\n" << sdp_str;

  flatbuffers::FlatBufferBuilder fbb(512);
  flatbuffers::Offset<WebSocketSdp> sdp_fb =
      CreateWebSocketSdpDirect(fbb, SdpType::OFFER, sdp_str.c_str());
  flatbuffers::Offset<WebSocketMessage> answer_message =
      CreateWebSocketMessage(fbb, Payload::WebSocketSdp, sdp_fb.Union());
  fbb.Finish(answer_message);

  server_->execute(std::make_shared<UpdateData>(sock_, fbb.Release()));
}

void Connection::OnNegotiationNeeded() {
  LOG(INFO) << "OnNegotiationNeeded";

  GstPromise *promise;
  promise = gst_promise_new_with_change_func(Connection::OnOfferCreatedCallback,
                                             static_cast<gpointer>(this), NULL);
  g_signal_emit_by_name(G_OBJECT(webrtcbin_), "create-offer", NULL, promise);
}

void Connection::OnIceCandidate(guint mline_index, gchar *candidate) {
  LOG(INFO) << "OnIceCandidate";

  flatbuffers::FlatBufferBuilder fbb(512);

  auto ice_fb_builder = WebSocketIce::Builder(fbb);
  ice_fb_builder.add_sdp_m_line_index(mline_index);
  ice_fb_builder.add_sdp_mid(fbb.CreateString("video0"));
  ice_fb_builder.add_candidate(
      fbb.CreateString(static_cast<char *>(candidate)));
  flatbuffers::Offset<WebSocketIce> ice_fb = ice_fb_builder.Finish();

  flatbuffers::Offset<WebSocketMessage> ice_message =
      CreateWebSocketMessage(fbb, Payload::WebSocketIce, ice_fb.Union());
  fbb.Finish(ice_message);

  server_->execute(std::make_shared<UpdateData>(sock_, fbb.Release()));

  g_signal_emit_by_name(webrtcbin_, "add-ice-candidate", mline_index,
                        candidate);
}

void Connection::HandleWebSocketData(const uint8_t *data, size_t /* size*/) {
  LOG(INFO) << "HandleWebSocketData";

  const WebSocketMessage *message =
      flatbuffers::GetRoot<WebSocketMessage>(data);

  switch (message->payload_type()) {
    case Payload::WebSocketSdp: {
      const WebSocketSdp *offer = message->payload_as_WebSocketSdp();
      if (offer->type() != SdpType::ANSWER) {
        LOG(WARNING) << "Expected SDP message type \"answer\"";
        break;
      }
      const flatbuffers::String *sdp_string = offer->payload();

      LOG(INFO) << "Received SDP:\n" << sdp_string->c_str();

      GstSDPMessage *sdp;
      GstSDPResult status = gst_sdp_message_new(&sdp);
      if (status != GST_SDP_OK) {
        LOG(WARNING) << "Could not create SDP message";
        break;
      }

      status = gst_sdp_message_parse_buffer((const guint8 *)sdp_string->c_str(),
                                            sdp_string->size(), sdp);

      if (status != GST_SDP_OK) {
        LOG(WARNING) << "Could not parse SDP string";
        break;
      }

      std::unique_ptr<GstWebRTCSessionDescription,
                      decltype(&gst_webrtc_session_description_free)>
          answer(gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER,
                                                    sdp),
                 &gst_webrtc_session_description_free);
      std::unique_ptr<GstPromise, decltype(&gst_promise_unref)> promise(
          gst_promise_new(), &gst_promise_unref);
      g_signal_emit_by_name(webrtcbin_, "set-remote-description", answer.get(),
                            promise.get());
      gst_promise_interrupt(promise.get());

      break;
    }
    case Payload::WebSocketIce: {
      const WebSocketIce *ice = message->payload_as_WebSocketIce();
      if (!ice->has_candidate() || ice->candidate()->size() == 0) {
        LOG(WARNING) << "Received ICE message without candidate";
        break;
      }

      const gchar *candidate =
          static_cast<const gchar *>(ice->candidate()->c_str());
      guint mline_index = ice->sdp_m_line_index();

      LOG(INFO) << "Received ICE candidate with mline index " << mline_index
                << "; candidate: " << candidate;

      g_signal_emit_by_name(webrtcbin_, "add-ice-candidate", mline_index,
                            candidate);

      break;
    }
    default:
      break;
  }
}

void RegisterPlugins() {
  GST_PLUGIN_STATIC_REGISTER(app);
  GST_PLUGIN_STATIC_REGISTER(coreelements);
  GST_PLUGIN_STATIC_REGISTER(dtls);
  GST_PLUGIN_STATIC_REGISTER(nice);
  GST_PLUGIN_STATIC_REGISTER(rtp);
  GST_PLUGIN_STATIC_REGISTER(rtpmanager);
  GST_PLUGIN_STATIC_REGISTER(srtp);
  GST_PLUGIN_STATIC_REGISTER(webrtc);
  GST_PLUGIN_STATIC_REGISTER(video4linux2);
  GST_PLUGIN_STATIC_REGISTER(videoconvert);
  GST_PLUGIN_STATIC_REGISTER(videoparsersbad);
  GST_PLUGIN_STATIC_REGISTER(videorate);
  GST_PLUGIN_STATIC_REGISTER(videoscale);
  GST_PLUGIN_STATIC_REGISTER(videotestsrc);
  GST_PLUGIN_STATIC_REGISTER(x264);
}

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  findEmbeddedContent("");

  std::string openssl_env = "OPENSSL_CONF=\"\"";
  putenv(const_cast<char *>(openssl_env.c_str()));

  putenv(const_cast<char *>("GST_REGISTRY_DISABLE=yes"));

  gst_init(&argc, &argv);
  RegisterPlugins();

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);
  aos::ShmEventLoop event_loop(&config.message());

  {
    aos::GlibMainLoop main_loop(&event_loop);

    seasocks::Server server(::std::shared_ptr<seasocks::Logger>(
        new ::aos::seasocks::SeasocksLogger(seasocks::Logger::Level::Info)));

    LOG(INFO) << "Serving from " << FLAGS_data_dir;

    auto websocket_handler =
        std::make_shared<WebsocketHandler>(&event_loop, &server);
    server.addWebSocketHandler("/ws", websocket_handler);

    server.startListening(1180);
    server.setStaticPath(FLAGS_data_dir.c_str());

    aos::internal::EPoll *epoll = event_loop.epoll();

    epoll->OnReadable(server.fd(), [&server] {
      CHECK(::seasocks::Server::PollResult::Continue == server.poll(0));
    });

    event_loop.Run();

    epoll->DeleteFd(server.fd());
    server.terminate();
  }

  gst_deinit();

  return 0;
}
