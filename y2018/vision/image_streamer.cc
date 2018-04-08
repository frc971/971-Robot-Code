#include "aos/vision/events/socket_types.h"
#include "aos/vision/image/reader.h"
#include "aos/vision/image/image_stream.h"
#include "aos/common/logging/implementations.h"
#include "aos/common/logging/logging.h"
#include "aos/vision/blob/codec.h"
#include <fstream>
#include <sys/stat.h>
#include <deque>
#include <string>

using aos::events::TCPServer;
using aos::events::DataSocket;
using aos::vision::Int32Codec;
using aos::vision::DataRef;

aos::vision::DataRef mjpg_header = "HTTP/1.0 200 OK\r\n"\
      "Server: YourServerName\r\n"\
      "Connection: close\r\n"\
      "Max-Age: 0\r\n"\
      "Expires: 0\r\n"\
      "Cache-Control: no-cache, private\r\n"\
      "Pragma: no-cache\r\n"\
      "Content-Type: multipart/x-mixed-replace; "\
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

class MjpegDataSocket : public aos::events::SocketConnection {
 public:

  MjpegDataSocket(aos::events::TCPServerBase *serv, int fd)
      : aos::events::SocketConnection(serv, fd) {
    SetEvents(EPOLLOUT | EPOLLET);
  }

  ~MjpegDataSocket() { printf("Closed connection on descriptor %d\n", fd()); }

  void DirectEvent(uint32_t events) override {
    if (events & EPOLLOUT) {
      NewDataToSend();
      events &= ~EPOLLOUT;
    }
    if (events) {
      aos::events::EpollEvent::DirectEvent(events);
    }
  }

  void ReadEvent() override {
    // Ignore reads, but don't leave them pending.
    ssize_t count;
    char buf[512];
    while (true) {
      count = read(fd(), &buf, sizeof buf);
      if (count <= 0) {
        if (errno != EAGAIN) {
          CloseConnection();
          return;
        }
        break;
      } else if (!ready_to_recieve_) {
        // This 4 should match "\r\n\r\n".length();
        if (match_i_ >= 4) {
          printf("reading after last match\n");
          continue;
        }
        for (char c : aos::vision::DataRef(&buf[0], count)) {
          if (c == "\r\n\r\n"[match_i_]) {
            ++match_i_;
            if (match_i_ >= 4) {
              if (!ready_to_recieve_) {
                ready_to_recieve_ = true;
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
    if (!output_buffer_.empty() || !ready_to_recieve_) return;
    sending_frame_ = frame;
    aos::vision::DataRef data = frame->data;

    size_t n_written = snprintf(data_header_tmp_, sizeof(data_header_tmp_),
                                "--boundary\r\n"\
                                "Content-type: image/jpg\r\n"\
                                "Content-Length: %zu\r\n\r\n", data.size());
    // This should never happen because the buffer should be properly sized.
    if (n_written == sizeof(data_header_tmp_)) {
      fprintf(stderr, "wrong sized buffer\n");
      exit(-1);
    }
    output_buffer_.push_back(aos::vision::DataRef(data_header_tmp_, n_written));
    output_buffer_.push_back(data);
    output_buffer_.push_back("\r\n\r\n");
    NewDataToSend();
  }

  void NewFrame(std::shared_ptr<Frame> frame) {
    RasterFrame(std::move(frame));
  }

  void NewDataToSend() {
    while (!output_buffer_.empty()) {
      auto& data = *output_buffer_.begin();

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

  bool ready_to_recieve_ = false;
  void CloseConnection() {
    loop()->Delete(this);
    close(fd());
    delete this;
  }
  size_t match_i_ = 0;
};

using namespace aos::vision;
class CameraStream : public ImageStreamEvent {
 public:
  CameraStream(aos::vision::CameraParams params, 
               const std::string &fname, TCPServer<MjpegDataSocket>* tcp_serv)
      : ImageStreamEvent(fname, params), tcp_serv_(tcp_serv),
        log_("./logging/blob_record_", ".dat") {}      

  void ProcessImage(DataRef data, aos::monotonic_clock::time_point tp) {
    (void)tp;
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
      log_.WriteLogEntry(log_record);
      sampling = 0;
    }

    auto frame = std::make_shared<Frame>(Frame{std::string(data)});
    tcp_serv_->Broadcast([frame](MjpegDataSocket* event) {
                         event->NewFrame(frame);
                         });
  }
 private:
  int sampling = 0;
  TCPServer<MjpegDataSocket>* tcp_serv_;
  BlobLog log_;
};

int main() {
  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stderr));

  TCPServer<MjpegDataSocket> tcp_serv_(80);
  aos::vision::CameraParams params;
  params.set_exposure(100);
  params.set_width(320);
  params.set_height(240);
  CameraStream camera(params, "/dev/video0", &tcp_serv_);

  aos::events::EpollLoop loop;
  loop.Add(&tcp_serv_);
  loop.Add(&camera);

  printf("Running Camera\n");
  loop.Run();
}
