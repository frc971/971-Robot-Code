#ifndef AOS_LINUX_CODE_CAMERA_CAMERA_BUFFERS_H_
#define AOS_LINUX_CODE_CAMERA_CAMERA_BUFFERS_H_

#include <sys/socket.h>
#include <sys/un.h>

#include <string>

#include "aos/linux_code/ipc_lib/queue.h"
#include "aos/common/type_traits.h"
#include "aos/linux_code/ipc_lib/unique_message_ptr.h"

namespace aos {
namespace camera {

class Reader;
class Buffers {
  // It has to do a lot of the same things as all the other ones, but it gets
  // the information from different places (some of it gets sent out by it).
  friend class Reader;

  // Not an abstract name so that an existing one can just be unlinked without
  // disturbing it if necessary (like with shm_link).
  static const std::string kFDServerName;
  // How many v4l2 buffers and other things that depend on that.
  static const unsigned int kNumBuffers = 10;
  // How many fds to transfer from the fd server.
  // Used to make it clear which 1s are 1 and which are this.
  static const size_t kNumFDs = 1;
  // Creates a socket and calls either bind or connect.
  // Returns a bound/connected socket or -1 (the reason for which will already
  // have been logged at ERROR).
  static int CreateSocket(int (*bind_connect)(int, const sockaddr *, socklen_t));

  // File descriptor connected to the fd server. Used for detecting if the
  // camera reading process has died.
  // A value of -2 means don't check.
  const int server_;
  // Gets the fd (using server_).
  int FetchFD();
  // File descriptor for the v4l2 device (that's valid in this process of
  // course).
  const int fd_;

  struct Buffer;
  // Buffer[kNumBuffers]
  Buffer *buffers_;
  struct Message {
    uint32_t index;
    uint32_t bytesused;
    timeval timestamp;
    uint32_t sequence;
  };
  static_assert(shm_ok<Message>::value, "it's going through queues");

  // NULL for the Reader one.
  RawQueue *const queue_;
  // The current one. Sometimes NULL.
  unique_message_ptr<const Message> message_;

  static const std::string kQueueName;
  // Make the actual mmap calls.
  // Called by Buffers() automatically.
  void MMap();

 public:
  Buffers();
  // Will clean everything up.
  // So that HTTPStreamer can create/destroy one for each client to make it
  // simpler.
  ~Buffers();

  // Retrieves the next image. Will return the current one if it hasn't yet.
  // Calls Release() at the beginning.
  // NOTE: this means that the caller can't keep using references to the old
  // return value after calling this function again
  // block is whether to return NULL or wait for a new one
  // the last 3 output parameters will be filled in straight from the
  // v4l2_buffer if they're not NULL
  // (see <http://v4l2spec.bytesex.org/spec/x5953.htm#V4L2-BUFFER> for details)
  // NOTE: guaranteed to return a valid pointer if block is true
  const void *GetNext(bool block,
                uint32_t *bytesused, timeval *timestamp, uint32_t *sequence);
  // Releases the most recent frame buffer. Automatically called by GetNext and
  // the destructor. Safe to call multiple times without getting frames in
  // between.
  void Release();

  // How big images are.
  static const int32_t kWidth = 320, kHeight = 240;
};

} // namespace camera
} // namespace aos

#endif
