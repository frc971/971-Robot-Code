#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>

#include <string>
#include <inttypes.h>

#include "aos/atom_code/init.h"
#include "aos/atom_code/camera/V4L2.h"
#include "aos/atom_code/camera/Buffers.h"
#include "aos/common/logging/logging.h"
#include "aos/atom_code/ipc_lib/queue.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace aos {
namespace camera {

class Reader {
  static const char *const dev_name;

  // of the camera
  int fd_;
  // the bound socket listening for fd requests
  int server_fd_;

  RawQueue *queue_, *recycle_queue_;
  // the number of buffers currently queued in v4l2
  uint32_t queued_;
 public:
  Reader() {
    struct stat st; 
    if (stat(dev_name, &st) == -1) {
      LOG(FATAL, "Cannot identify '%s' because of %d: %s\n",
              dev_name, errno, strerror(errno));
    }
    if (!S_ISCHR(st.st_mode)) {
      LOG(FATAL, "%s is no device\n", dev_name);
    }

    fd_ = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
    if (fd_ == -1) {
      LOG(FATAL, "Cannot open '%s' because of %d: %s\n",
              dev_name, errno, strerror(errno));
    }

    queue_ = RawQueue::Fetch(Buffers::kQueueName.c_str(),
                          sizeof(Buffers::Message), 971, 1,
                          1, Buffers::kNumBuffers, &recycle_queue_);
    // read off any existing recycled messages
    while (recycle_queue_->ReadMessage(RawQueue::kNonBlock) != NULL);
    queued_ = 0;

    InitServer();
    Init();
  }
 private:
  void InitServer() {
    if (unlink(Buffers::kFDServerName.c_str()) == -1 && errno != ENOENT) {
      LOG(WARNING, "unlink(kFDServerName(='%s')) failed with %d: %s\n",
          Buffers::kFDServerName.c_str(), errno, strerror(errno));
    }
    if ((server_fd_ = Buffers::CreateSocket(bind)) == -1) {
      LOG(FATAL, "creating the IPC socket failed\n");
    }
    if (listen(server_fd_, 10) == -1) {
      LOG(FATAL, "listen(%d, 10) failed with %d: %s\n",
          server_fd_, errno, strerror(errno));
    }
  }
  void SendFD(const int sock) {
    int myfds[Buffers::kNumFDs]; /* Contains the file descriptors to pass. */
    myfds[0] = fd_;
    char buf[CMSG_SPACE(sizeof(myfds))];  /* ancillary data buffer */

    iovec data;
    memset(&data, 0, sizeof(data));
    char dummy = 'A';
    data.iov_base = &dummy;
    data.iov_len = sizeof(dummy);
    msghdr msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_iov = &data;
    msg.msg_iovlen = 1;
    msg.msg_control = buf;
    msg.msg_controllen = sizeof(buf);
    cmsghdr *const cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(sizeof(myfds));
    /* Initialize the payload: */
    memcpy(CMSG_DATA(cmsg), myfds, sizeof(myfds));
    if (sendmsg(sock, &msg, 0) == -1) {
      LOG(ERROR, "sendmsg(%d, %p, 0) failed with %d: %s\n",
          sock, &msg, errno, strerror(errno));
    }
    // leave it open so that the other end can tell if this process dies
  }

#if 0
  // if we ever do want to do any of these things, this is how
  void Stop() {
    const v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMOFF, &type) == -1) {
      errno_exit("VIDIOC_STREAMOFF");
    }
  }
  void Close() {
    if (close(fd_) == -1)
      errno_exit("close");
    fd_ = -1;
  }
#endif

  void QueueBuffer(v4l2_buffer *buf) {
    if (xioctl(fd_, VIDIOC_QBUF, buf) == -1) {
      LOG(WARNING, "ioctl VIDIOC_QBUF(%d, %p) failed with %d: %s."
          " losing buf #%" PRIu32 "\n",
          fd_, &buf, errno, strerror(errno), buf->index);
    } else {
      LOG(DEBUG, "put buf #%" PRIu32 " into driver's queue\n", buf->index);
      ++queued_;
    }
  }
  void ReadFrame() {
    v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    
    const Buffers::Message *read;
    do {
      read = static_cast<const Buffers::Message *>(
          // we block waiting for one if we can't dequeue one without leaving
          // the driver <= 2 (to be safe)
          recycle_queue_->ReadMessage((queued_ <= 2) ?
                                      RawQueue::kBlock : RawQueue::kNonBlock));
      if (read != NULL) {
        buf.index = read->index;
        recycle_queue_->FreeMessage(read);
        QueueBuffer(&buf);
      }
    } while (read != NULL);

    if (xioctl(fd_, VIDIOC_DQBUF, &buf) == -1) {
      if (errno != EAGAIN) {
        LOG(ERROR, "ioctl VIDIOC_DQBUF(%d, %p) failed with %d: %s\n",
            fd_, &buf, errno, strerror(errno));
      }
      return;
    }
    --queued_;
    if (buf.index >= Buffers::kNumBuffers) {
      LOG(ERROR, "buf.index (%" PRIu32 ") is >= kNumBuffers (%u)\n",
          buf.index, Buffers::kNumBuffers);
      return;
    }

    Buffers::Message *const msg = static_cast<Buffers::Message *>(
        queue_->GetMessage());
    if (msg == NULL) {
      LOG(WARNING,
          "couldn't get a message to send buf #%" PRIu32 " from queue %p."
          " re-queueing now\n", buf.index, queue_);
      QueueBuffer(&buf);
      return;
    }
    msg->index = buf.index;
    msg->bytesused = buf.bytesused;
    memcpy(&msg->timestamp, &buf.timestamp, sizeof(msg->timestamp));
    msg->sequence = buf.sequence;
    if (!queue_->WriteMessage(msg, RawQueue::kOverride)) {
      LOG(WARNING,
          "sending message %p with buf #%" PRIu32 " to queue %p failed."
          " re-queueing now\n", msg, buf.index, queue_);
      QueueBuffer(&buf);
      return;
    } else {
      LOG(DEBUG, "sent message off to queue %p with buffer #%" PRIu32 "\n",
          queue_, buf.index);
    }
  }

  void init_mmap() {
    v4l2_requestbuffers req;
    CLEAR(req);
    req.count = Buffers::kNumBuffers;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &req) == -1) {
      if (EINVAL == errno) {
        LOG(FATAL, "%s does not support memory mapping\n", dev_name);
      } else {
        LOG(FATAL, "ioctl VIDIOC_REQBUFS(%d, %p) failed with %d: %s\n",
            fd_, &req, errno, strerror(errno));
      }
    }
    queued_ = Buffers::kNumBuffers;
    if (req.count < Buffers::kNumBuffers) {
      LOG(FATAL, "Insufficient buffer memory on %s\n", dev_name);
    }
  }

  // Sets one of the camera's user-control values.
  // Prints the old and new values.
  // Just prints a message if the camera doesn't support this control or value.
  bool SetCameraControl(uint32_t id, const char *name, int value) {
    struct v4l2_control getArg = {id, 0U};
    int r = xioctl(fd_, VIDIOC_G_CTRL, &getArg);
    if (r == 0) {
      if (getArg.value == value) {
        printf("Camera control %s was already %d\n", name, getArg.value);
        return true;
      }
    } else if (errno == EINVAL) {
      printf("Camera control %s is invalid\n", name);
      errno = 0;
      return false;
    }

    struct v4l2_control setArg = {id, value};
    r = xioctl(fd_, VIDIOC_S_CTRL, &setArg);
    if (r == 0) {
      printf("Set camera control %s from %d to %d\n", name, getArg.value, value);
      return true;
    }

    printf("Couldn't set camera control %s to %d: %s (errno %d)\n",
        name, value, strerror(errno), errno);
    errno = 0;
    return false;
  }

  void Init() {
    v4l2_capability cap;
    if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) == -1) {
      if (EINVAL == errno) {
        LOG(FATAL, "%s is no V4L2 device\n",
                dev_name);
      } else {
        LOG(FATAL, "ioctl VIDIOC_QUERYCAP(%d, %p) failed with %d: %s\n",
            fd_, &cap, errno, strerror(errno));
      }
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
      LOG(FATAL, "%s is no video capture device\n",
              dev_name);
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
      LOG(FATAL, "%s does not support streaming i/o\n",
              dev_name);
    }

    /* Select video input, video standard and tune here. */

    v4l2_cropcap cropcap;
    CLEAR(cropcap);
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_CROPCAP, &cropcap) == 0) {
      v4l2_crop crop;
      crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      crop.c = cropcap.defrect; /* reset to default */

      if (xioctl(fd_, VIDIOC_S_CROP, &crop) == -1) {
        switch (errno) {
          case EINVAL:
            /* Cropping not supported. */
            break;
          default:
            /* Errors ignored. */
            break;
        }
      }
    } else {        
      /* Errors ignored. */
    }

    v4l2_format fmt;
    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = Buffers::kWidth; 
    fmt.fmt.pix.height = Buffers::kHeight;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    //fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) == -1) {
      LOG(FATAL, "ioctl VIDIC_S_FMT(%d, %p) failed with %d: %s\n",
          fd_, &fmt, errno, strerror(errno));
    }
    /* Note VIDIOC_S_FMT may change width and height. */

    /* Buggy driver paranoia. */
    unsigned int min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
      fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
      fmt.fmt.pix.sizeimage = min;

    if (!SetCameraControl(V4L2_CID_EXPOSURE_AUTO,
                          "V4L2_CID_EXPOSURE_AUTO", V4L2_EXPOSURE_MANUAL)) {
      LOG(FATAL, "Failed to set exposure\n");
    }

    if (!SetCameraControl(V4L2_CID_EXPOSURE_ABSOLUTE,
                          "V4L2_CID_EXPOSURE_ABSOLUTE", 65)) {
      LOG(FATAL, "Failed to set exposure\n");
    }

    if (!SetCameraControl(V4L2_CID_BRIGHTNESS, "V4L2_CID_BRIGHTNESS", 128)) {
      LOG(FATAL, "Failed to set up camera\n");
    }

    if (!SetCameraControl(V4L2_CID_GAIN, "V4L2_CID_GAIN", 0)) {
      LOG(FATAL, "Failed to set up camera\n");
    }

#if 0
    // set framerate
    struct v4l2_streamparm *setfps;
    setfps = (struct v4l2_streamparm *) calloc(1, sizeof(struct v4l2_streamparm));
    memset(setfps, 0, sizeof(struct v4l2_streamparm));
    setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    setfps->parm.capture.timeperframe.numerator = 1;
    setfps->parm.capture.timeperframe.denominator = 20;
    if (xioctl(fd_, VIDIOC_S_PARM, setfps) == -1) {
      LOG(ERROR, "ioctl VIDIOC_S_PARM(%d, %p) failed with %d: %s\n",
          fd_, setfps, errno, strerror(errno));
      exit(EXIT_FAILURE);
    }
    LOG(INFO, "framerate ended up at %d/%d\n",
        setfps->parm.capture.timeperframe.numerator,
        setfps->parm.capture.timeperframe.denominator);
#endif

    init_mmap();
  }

  void Start() {
    LOG(DEBUG, "queueing buffers for the first time\n");
    v4l2_buffer buf;
    for (unsigned int i = 0; i < Buffers::kNumBuffers; ++i) {
      CLEAR(buf);
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      QueueBuffer(&buf);
    }
    LOG(DEBUG, "done with first queue\n");

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
      LOG(FATAL, "ioctl VIDIOC_STREAMON(%d, %p) failed with %d: %s\n",
          fd_, &type, errno, strerror(errno));
    }
  }

 public:
  void Run() {
    Start();

    fd_set fds;
    timeval tv;
    while (true) {
      // HAVE TO DO THIS EACH TIME THROUGH THE LOOP
      tv.tv_sec = 2;
      tv.tv_usec = 0;

      FD_ZERO(&fds);
      FD_SET(fd_, &fds);
      FD_SET(server_fd_, &fds);
      switch (select(std::max(fd_, server_fd_) + 1, &fds, NULL, NULL, &tv)) {
        case -1:
          if (errno != EINTR) {
            LOG(ERROR, "select(%d, %p, NULL, NULL, %p) failed with %d: %s\n",
                std::max(fd_, server_fd_) + 1, &fds, &tv, errno, strerror(errno));
          }
          continue;
        case 0:
          LOG(WARNING, "select timed out\n");
          continue;
      }

      if (FD_ISSET(fd_, &fds)) {
        LOG(INFO, "Got a frame\n");
        ReadFrame();
      }
      if (FD_ISSET(server_fd_, &fds)) {
        const int sock = accept4(server_fd_, NULL, NULL, SOCK_NONBLOCK);
        if (sock == -1) {
          LOG(ERROR, "accept4(%d, NULL, NULL, SOCK_NONBLOCK(=%d) failed with %d: %s\n",
              server_fd_, SOCK_NONBLOCK, errno, strerror(errno));
        } else {
          SendFD(sock);
        }
      }
    }
  }
};
const char *const Reader::dev_name = "/dev/video0";

} // namespace camera
} // namespace aos

int main() {
  ::aos::InitNRT();
  ::aos::camera::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
