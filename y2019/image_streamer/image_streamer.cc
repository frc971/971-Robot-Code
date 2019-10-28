#include "aos/vision/image/image_stream.h"

#include <sys/stat.h>
#include <deque>
#include <fstream>
#include <string>

#include "aos/logging/implementations.h"
#include "aos/logging/logging.h"
#include "aos/vision/blob/codec.h"
#include "aos/vision/events/socket_types.h"
#include "aos/vision/events/udp.h"
#include "aos/vision/image/reader.h"
#include "gflags/gflags.h"
#include "y2019/image_streamer/flip_image.h"
#include "y2019/vision.pb.h"

using ::aos::events::DataSocket;
using ::aos::events::RXUdpSocket;
using ::aos::events::TCPServer;
using ::aos::vision::DataRef;
using ::aos::vision::Int32Codec;
using ::aos::monotonic_clock;
using ::y2019::VisionControl;

DEFINE_string(roborio_ip, "10.9.71.2", "RoboRIO IP Address");
DEFINE_string(log, "",
              "If non-empty, log images to the specified prefix with the image "
              "index appended to the filename");
DEFINE_bool(single_camera, true, "If true, only use video0");
DEFINE_int32(camera0_exposure, 600, "Exposure for video0");
DEFINE_int32(camera1_exposure, 600, "Exposure for video1");

aos::vision::DataRef mjpg_header =
    "HTTP/1.0 200 OK\r\n"
    "Server: YourServerName\r\n"
    "Connection: close\r\n"
    "Max-Age: 0\r\n"
    "Expires: 0\r\n"
    "Cache-Control: no-cache, private\r\n"
    "Pragma: no-cache\r\n"
    "Content-Type: multipart/x-mixed-replace; "
    "boundary=--boundary\r\n\r\n";

struct Frame {
  std::string data;
};

inline bool FileExist(const std::string &name) {
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

class BlobLog {
 public:
  explicit BlobLog(const char *prefix, const char *extension) {
    int index = 0;
    while (true) {
      std::string file = prefix + std::to_string(index) + extension;
      if (FileExist(file)) {
        index++;
      } else {
        printf("Logging to file (%s)\n", file.c_str());
        ofst_.open(file);
        assert(ofst_.is_open());
        break;
      }
    }
  }

  ~BlobLog() { ofst_.close(); }

  void WriteLogEntry(DataRef data) { ofst_.write(&data[0], data.size()); }

 private:
  std::ofstream ofst_;
};

class UdpClient : public ::aos::events::EpollEvent {
 public:
  UdpClient(int port, ::std::function<void(void *, size_t)> callback)
      : ::aos::events::EpollEvent(RXUdpSocket::SocketBindListenOnPort(port)),
        callback_(callback) {}

 private:
  ::std::function<void(void *, size_t)> callback_;

  void ReadEvent() override {
    char data[1024];
    size_t received_data_size = Recv(data, sizeof(data));
    callback_(data, received_data_size);
  }

  size_t Recv(void *data, int size) {
    return AOS_PCHECK(recv(fd(), static_cast<char *>(data), size, 0));
  }
};

// TODO(aschuh & michael) Pull this out.
template <typename PB>
class ProtoUdpClient : public UdpClient {
 public:
  ProtoUdpClient(int port, ::std::function<void(const PB &)> proto_callback)
      : UdpClient(port, ::std::bind(&ProtoUdpClient::ReadData, this,
                                    ::std::placeholders::_1,
                                    ::std::placeholders::_2)),
        proto_callback_(proto_callback) {}

 private:
  ::std::function<void(const PB &)> proto_callback_;

  void ReadData(void *data, size_t size) {
    PB pb;
    // TODO(Brian): Do something useful if parsing fails.
    pb.ParseFromArray(data, size);
    proto_callback_(pb);
  }
};

class MjpegDataSocket : public aos::events::SocketConnection {
 public:
  MjpegDataSocket(aos::events::TCPServerBase *server, int fd)
      : aos::events::SocketConnection(server, fd) {
    SetEvents(EPOLLOUT | EPOLLET);
  }

  ~MjpegDataSocket() { printf("Closed connection on descriptor %d\n", fd()); }

  void DirectEvent(uint32_t events) override {
    if (events & EPOLLOUT) {
      NewDataToSend();
      events &= ~EPOLLOUT;
    }
    // Other end hung up.  Ditch the connection.
    if (events & EPOLLHUP) {
      CloseConnection();
      events &= ~EPOLLHUP;
      return;
    }
    if (events) {
      aos::events::EpollEvent::DirectEvent(events);
    }
  }

  void ReadEvent() override {
    ssize_t count;
    char buf[512];
    while (true) {
      // Always read everything so epoll won't return immediately.
      count = read(fd(), &buf, sizeof buf);
      if (count <= 0) {
        if (errno != EAGAIN) {
          CloseConnection();
          return;
        }
        break;
      } else if (!ready_to_receive_) {
        // This 4 should match "\r\n\r\n".length();
        if (match_i_ >= 4) {
          printf("reading after last match\n");
          continue;
        }
        for (char c : aos::vision::DataRef(&buf[0], count)) {
          if (c == "\r\n\r\n"[match_i_]) {
            ++match_i_;
            if (match_i_ >= 4) {
              if (!ready_to_receive_) {
                ready_to_receive_ = true;
                RasterHeader();
              }
            }
          } else if (match_i_ != 0) {
            if (c == '\r') match_i_ = 1;
          }
        }
      }
    }
  }

  void RasterHeader() {
    output_buffer_.push_back(mjpg_header);
    NewDataToSend();
  }

  void RasterFrame(std::shared_ptr<Frame> frame) {
    if (!output_buffer_.empty() || !ready_to_receive_) return;
    sending_frame_ = frame;
    aos::vision::DataRef data = frame->data;

    size_t n_written = snprintf(data_header_tmp_, sizeof(data_header_tmp_),
                                "--boundary\r\n"
                                "Content-type: image/jpg\r\n"
                                "Content-Length: %zu\r\n\r\n",
                                data.size());
    // This should never happen because the buffer should be properly sized.
    if (n_written == sizeof(data_header_tmp_)) {
      fprintf(stderr, "wrong sized buffer\n");
      exit(-1);
    }
    AOS_LOG(INFO, "Frame size in bytes: data.size() = %zu\n", data.size());
    output_buffer_.push_back(aos::vision::DataRef(data_header_tmp_, n_written));
    output_buffer_.push_back(data);
    output_buffer_.push_back("\r\n\r\n");
    NewDataToSend();
  }

  void NewFrame(std::shared_ptr<Frame> frame) { RasterFrame(std::move(frame)); }

  void NewDataToSend() {
    while (!output_buffer_.empty()) {
      auto &data = *output_buffer_.begin();

      while (!data.empty()) {
        int len = send(fd(), data.data(), data.size(), MSG_NOSIGNAL);
        if (len == -1) {
          if (errno == EAGAIN) {
            // Next thinggy will pick this up.
            return;
          } else {
            CloseConnection();
            return;
          }
        } else {
          data.remove_prefix(len);
        }
      }
      output_buffer_.pop_front();
    }
  }

 private:
  char data_header_tmp_[512];
  std::shared_ptr<Frame> sending_frame_;
  std::deque<aos::vision::DataRef> output_buffer_;

  bool ready_to_receive_ = false;
  void CloseConnection() {
    loop()->Delete(this);
    close(fd());
    delete this;
  }
  size_t match_i_ = 0;
};

class CameraStream : public ::aos::vision::ImageStreamEvent {
 public:
  CameraStream(::aos::vision::CameraParams params, const ::std::string &fname,
               TCPServer<MjpegDataSocket> *tcp_server, bool log,
               ::std::function<void()> frame_callback)
      : ImageStreamEvent(fname, params),
        tcp_server_(tcp_server),
        frame_callback_(frame_callback) {
    if (log) {
      log_.reset(new BlobLog(FLAGS_log.c_str(), ".dat"));
    }
  }

  void set_active(bool active) { active_ = active; }

  void set_flip(bool flip) { flip_ = flip; }

  bool active() const { return active_; }

  void ProcessImage(DataRef data,
                    monotonic_clock::time_point /*monotonic_now*/) {
    ++sampling;
    // 20 is the sampling rate.
    if (sampling == 20) {
      int tmp_size = data.size() + sizeof(int32_t);
      char *buf;
      std::string log_record;
      log_record.resize(tmp_size, 0);
      {
        buf = Int32Codec::Write(&log_record[0], tmp_size);
        data.copy(buf, data.size());
      }
      if (log_) {
        log_->WriteLogEntry(log_record);
      }
      sampling = 0;
    }

    ::std::string image_out;

    unsigned int out_size = image_buffer_out_.size();
    flip_image(data.data(), data.size(), &image_buffer_out_[0], &out_size,
               flip_);
    image_out.assign(&image_buffer_out_[0], &image_buffer_out_[out_size]);

    if (active_) {
      auto frame = std::make_shared<Frame>(Frame{image_out});
      tcp_server_->Broadcast(
          [frame](MjpegDataSocket *event) { event->NewFrame(frame); });
    }
    frame_callback_();
  }

 private:
  int sampling = 0;
  TCPServer<MjpegDataSocket> *tcp_server_;
  ::std::unique_ptr<BlobLog> log_;
  ::std::function<void()> frame_callback_;
  bool active_ = false;
  bool flip_ = false;
  std::array<JOCTET, 100000> image_buffer_out_;
};

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stderr));
  TCPServer<MjpegDataSocket> tcp_server_(80);
  aos::vision::CameraParams params0;
  params0.set_exposure(FLAGS_camera0_exposure);
  params0.set_brightness(-40);
  params0.set_width(320);
  // params0.set_fps(10);
  params0.set_height(240);

  aos::vision::CameraParams params1 = params0;
  params1.set_exposure(FLAGS_camera1_exposure);

  ::y2019::VisionStatus vision_status;
  AOS_LOG(INFO,
          "The UDP socket should be on port 5001 to 10.9.71.2 for "
          "the competition robot.\n");
  AOS_LOG(INFO, "Starting UDP socket on port 5001 to %s\n",
          FLAGS_roborio_ip.c_str());
  ::aos::events::ProtoTXUdpSocket<::y2019::VisionStatus> status_socket(
      FLAGS_roborio_ip.c_str(), 5001);

  ::std::unique_ptr<CameraStream> camera1;
  ::std::unique_ptr<CameraStream> camera0(new CameraStream(
      params0, "/dev/video0", &tcp_server_, !FLAGS_log.empty(),
      [&camera0, &status_socket, &vision_status]() {
        vision_status.set_low_frame_count(vision_status.low_frame_count() + 1);
        AOS_LOG(INFO, "Got a frame cam0\n");
        if (camera0->active()) {
          status_socket.Send(vision_status);
        }
      }));
  if (!FLAGS_single_camera) {
    camera1.reset(new CameraStream(
        params1, "/dev/video1", &tcp_server_, false,
        [&camera1, &status_socket, &vision_status]() {
          vision_status.set_high_frame_count(vision_status.high_frame_count() +
                                             1);
          AOS_LOG(INFO, "Got a frame cam1\n");
          if (camera1->active()) {
            status_socket.Send(vision_status);
          }
        }));
  }

  ProtoUdpClient<VisionControl> udp_client(
      5000, [&camera0, &camera1](const VisionControl &vision_control) {
        bool cam0_active = false;
        camera0->set_flip(vision_control.flip_image());
        if (camera1) {
          camera1->set_flip(vision_control.flip_image());
          cam0_active = !vision_control.high_video();
          camera0->set_active(!vision_control.high_video());
          camera1->set_active(vision_control.high_video());
        } else {
          cam0_active = true;
          camera0->set_active(true);
        }
        AOS_LOG(INFO, "Got control packet, cam%d active\n",
                cam0_active ? 0 : 1);
      });

  // Default to camera0
  camera0->set_active(true);
  if (camera1) {
    camera1->set_active(false);
  }

  aos::events::EpollLoop loop;
  loop.Add(&tcp_server_);
  loop.Add(camera0.get());
  if (camera1) {
    loop.Add(camera1.get());
  }
  loop.Add(&udp_client);

  printf("Running Camera\n");
  loop.Run();
}
