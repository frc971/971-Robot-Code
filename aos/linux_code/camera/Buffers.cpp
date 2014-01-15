#include "aos/linux_code/camera/Buffers.h"

#include <sys/mman.h>

#include "aos/linux_code/camera/V4L2.h"
#include "aos/common/logging/logging.h"

namespace aos {
namespace camera {

// Represents an actual v4l2 buffer.
struct Buffers::Buffer {
  void *start;
  size_t length; // for munmap
};
const std::string Buffers::kFDServerName("/tmp/aos_fd_server");
const std::string Buffers::kQueueName("CameraBufferQueue");

int Buffers::CreateSocket(int (*bind_connect)(int, const sockaddr *, socklen_t)) {
  union af_unix_sockaddr {
    sockaddr_un un;
    sockaddr addr;
  } addr;
  const int r = socket(AF_UNIX, SOCK_STREAM, 0);
  if (r == -1) {
    LOG(FATAL, "socket(AF_UNIX, SOCK_STREAM, 0) failed with %d: %s\n",
        errno, strerror(errno));
  }
  addr.un.sun_family = AF_UNIX;
  memset(addr.un.sun_path, 0, sizeof(addr.un.sun_path));
  strcpy(addr.un.sun_path, kFDServerName.c_str());
  if (bind_connect(r, &addr.addr, sizeof(addr.un)) == -1) {
    LOG(FATAL, "bind_connect(=%p)(%d, %p, %zd) failed with %d: %s\n",
        bind_connect, r, &addr.addr, sizeof(addr.un), errno, strerror(errno));
  }
  return r;
}

void Buffers::MMap() {
  buffers_ = new Buffer[kNumBuffers];
  v4l2_buffer buf;
  for (unsigned int n = 0; n < kNumBuffers; ++n) {
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1) {
      LOG(FATAL, "ioctl VIDIOC_QUERYBUF(%d, %p) failed with %d: %s\n",
          fd_, &buf, errno, strerror(errno));
    }
    buffers_[n].length = buf.length;
    buffers_[n].start = mmap(NULL, buf.length,
                             PROT_READ | PROT_WRITE, MAP_SHARED,
                             fd_, buf.m.offset);
    if (buffers_[n].start == MAP_FAILED) {
      LOG(FATAL, "mmap(NULL, %zd, PROT_READ | PROT_WRITE, MAP_SHARED, %d, %jd)"
          " failed with %d: %s\n", buf.length, fd_, static_cast<intmax_t>(buf.m.offset),
          errno, strerror(errno));
    }
  }
}

void Buffers::Release() {
  if (message_ != NULL) {
    queue_->FreeMessage(message_);
    message_ = NULL;
  }
}
const void *Buffers::GetNext(bool block,
                       uint32_t *bytesused, timeval *timestamp, uint32_t *sequence) {
  Release();

  // TODO(brians) make sure the camera reader process hasn't died
  do {
    if (block) {
      message_ = static_cast<const Message *>(queue_->ReadMessage(
              RawQueue::kPeek | RawQueue::kBlock));
    } else {
      static int index = 0;
      message_ = static_cast<const Message *>(queue_->ReadMessageIndex(
              RawQueue::kBlock, &index));
    }
  } while (block && message_ == NULL);
  if (message_ != NULL) {
    if (bytesused != NULL) memcpy(bytesused, &message_->bytesused, sizeof(*bytesused));
    if (timestamp != NULL) memcpy(timestamp, &message_->timestamp, sizeof(*timestamp));
    if (sequence != NULL) memcpy(sequence, &message_->sequence, sizeof(*sequence));
    return buffers_[message_->index].start;
  } else {
    return NULL;
  }
}

int Buffers::FetchFD() {
  int myfds[Buffers::kNumFDs]; // where to retrieve the fds into
  char buf[CMSG_SPACE(sizeof(myfds))]; // ancillary data buffer

  iovec data;
  memset(&data, 0, sizeof(data));
  char dummy;
  data.iov_base = &dummy;
  data.iov_len = sizeof(dummy);
  msghdr msg;
  memset(&msg, 0, sizeof(msg));
  msg.msg_iov = &data;
  msg.msg_iovlen = 1;
  msg.msg_control = buf;
  msg.msg_controllen = sizeof(buf);

  switch (recvmsg(server_, &msg, 0)) {
    case 0: // "the peer has performed an orderly shutdown"
      LOG(FATAL, "the fd server shut down (connected on %d)\n", server_);
    case -1:
      LOG(FATAL, "recvmsg(server_(=%d), %p, 0) failed with %d: %s\n",
          server_, &msg, errno, strerror(errno));
  }
  const cmsghdr *const cmsg = CMSG_FIRSTHDR(&msg);
  if (cmsg == NULL) {
    LOG(FATAL, "no headers in message\n");
  }
  if (cmsg->cmsg_len != CMSG_LEN(sizeof(myfds))) {
    LOG(FATAL, "got wrong size. got %d but expected %zd\n",
        cmsg->cmsg_len, CMSG_LEN(sizeof(myfds)));
  }
  if (cmsg->cmsg_level != SOL_SOCKET) {
    LOG(FATAL, "cmsg_level=%d. expected SOL_SOCKET(=%d)\n", cmsg->cmsg_level, SOL_SOCKET);
  }
  if (cmsg->cmsg_type != SCM_RIGHTS) {
    LOG(FATAL, "cmsg_type=%d. expected SCM_RIGHTS(=%d)\n", cmsg->cmsg_type, SCM_RIGHTS);
  }
  memcpy(myfds, CMSG_DATA(cmsg), sizeof(myfds));
  
  return myfds[0];
}
Buffers::Buffers() : server_(CreateSocket(connect)), fd_(FetchFD()), message_(NULL) {
  MMap();
  queue_ = RawQueue::Fetch(kQueueName.c_str(), sizeof(Message), 971, 1);
}

Buffers::~Buffers() {
  Release();

  for (unsigned i = 0; i < kNumBuffers; ++i) {
    if (munmap(buffers_[i].start, buffers_[i].length) == -1) {
      LOG(WARNING, "munmap(%p, %zd) for destruction failed with %d: %s\n",
          buffers_[i].start, buffers_[i].length, errno, strerror(errno));
    }
  }
  delete[] buffers_;

  if (close(fd_) == -1) {
    LOG(WARNING, "close(%d) for destruction failed with %d: %s\n",
        fd_, errno, strerror(errno));
  }
}

}  // namespace camera
}  // namespace aos
