#include "aos/vision/debug/debug_framework.h"

#include <gdk/gdk.h>
#include <gtk/gtk.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <string>

#include "aos/vision/blob/codec.h"
#include "aos/vision/events/tcp_client.h"

namespace aos {
namespace vision {

// Reads packets in the form:
// uint32 length
// string text (of size length)
class BufferedLengthDelimReader {
 public:
  union data_len {
    uint32_t len;
    char buf[4];
  };
  BufferedLengthDelimReader() {
    num_read_ = 0;
    img_read_ = -1;
  }
  template <typename Lamb>
  void up(int fd, Lamb lam) {
    // Drops older messages with this while loop until we've seeked to
    // be current.
    while (true) {
      ssize_t count;
      if (img_read_ < 0) {
        count = read(fd, &len_.buf[num_read_], sizeof(len_.buf) - num_read_);
        if (count < 0) break;
        num_read_ += count;
        if (num_read_ < 4) break;
        num_read_ = 0;
        img_read_ = 0;
        data_.clear();
        if (len_.len > 200000) {
          printf("bad size: %d\n", len_.len);
          exit(-1);
        }
        data_.resize(len_.len);
      } else {
        count = read(fd, &data_[img_read_], len_.len - img_read_);
        if (count < 0) break;
        img_read_ += count;
        if (img_read_ < (int)len_.len) return;
        std::swap(prev_data_, data_);
        has_prev_ = true;
        img_read_ = -1;
      }
    }
    if (has_prev_) {
      lam(DataRef{&prev_data_[0], len_.len});
      has_prev_ = false;
    }
  }

 private:
  data_len len_;
  int num_read_;
  bool has_prev_ = false;
  std::vector<char> prev_data_;
  std::vector<char> data_;
  int img_read_;
};

bool ParsePort(const std::string &port, int *portno) {
  if (port.empty()) return false;
  int value = 0;
  if (port[0] == '0') return false;
  for (char item : port) {
    if (item < '0' || item > '9') return false;
    value = value * 10 + (item - '0');
  }
  *portno = value;
  return true;
}

class TCPImageSource : public ImageSource {
 public:
  class Impl : public aos::events::TcpClient {
   public:
    Impl(const std::string &hostname, int portno,
         DebugFrameworkInterface *interface)
        : aos::events::TcpClient(hostname.c_str(), portno),
          interface_(interface) {}

    void ReadEvent() override {
      read_.up(fd(), [&](DataRef data) {
        BlobList blobl;
        const char *buf = data.data();
        buf += sizeof(uint32_t);

        ImageFormat fmt;
        Int64Codec::Read(&buf);
        fmt.w = Int32Codec::Read(&buf);
        fmt.h = Int32Codec::Read(&buf);
        buf = ParseBlobList(&blobl, buf);
        interface_->NewBlobList(blobl, fmt);
      });
      if (errno != EAGAIN) {
        fprintf(stderr, "disconnected\n");
        loop()->Delete(this);
      }
    }

    BufferedLengthDelimReader read_;
    DebugFrameworkInterface *interface_ = nullptr;
  };

  void Init(const std::string &addr_and_port,
            DebugFrameworkInterface *interface) override {
    auto it = addr_and_port.rfind(':');
    if (it == std::string::npos) {
      fprintf(stderr, "usage is: tcp:hostname:port\n");
      exit(-1);
    }
    auto hostname = addr_and_port.substr(0, it);
    auto port = addr_and_port.substr(it + 1);
    int portno = 0;
    if (!ParsePort(port, &portno)) {
      fprintf(stderr, "usage is: tcp:hostname:port\n");
      exit(-1);
    }

    impl_.reset(new Impl(hostname, portno, interface));

    interface->Loop()->Add(impl_.get());

    interface->InstallKeyPress([this](uint32_t /*keyval*/) {});
  }

  const char *GetHelpMessage() override {
    return &R"(
    format_spec is in ipaddr:port format.
    This viewer soure connects to a target_sender binary and views the live
    blob-stream.
)"[1];
  }

 private:
  std::unique_ptr<Impl> impl_;
};

REGISTER_IMAGE_SOURCE("tcp", TCPImageSource);

}  // namespace vision
}  // namespace aos
