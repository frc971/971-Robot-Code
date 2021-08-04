#ifndef AOS_NETWORK_SCTP_LIB_H_
#define AOS_NETWORK_SCTP_LIB_H_

#include <arpa/inet.h>
#include <netinet/sctp.h>

#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "aos/unique_malloc_ptr.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

namespace aos {
namespace message_bridge {

// Resolves a socket and returns the address.  This can be either an ipv4 or
// ipv6 address.
struct sockaddr_storage ResolveSocket(std::string_view host, int port);

// Returns a formatted version of the address.
std::string Address(const struct sockaddr_storage &sockaddr);
// Returns a formatted version of the address family.
std::string_view Family(const struct sockaddr_storage &sockaddr);

// Message received.
// This message is malloced bigger than needed and the extra space after it is
// the data.
struct Message {
  // Struct to let us force data to be well aligned.
  struct OveralignedChar {
    uint8_t data alignas(32);
  };

  // Headers.
  struct {
    struct sctp_rcvinfo rcvinfo;
  } header;

  // Address of the sender.
  struct sockaddr_storage sin;

  // Data type. Is it a block of data, or is it a struct sctp_notification?
  enum MessageType { kMessage, kNotification } message_type;

  size_t size = 0u;
  uint8_t *mutable_data() {
    return reinterpret_cast<uint8_t *>(&actual_data[0].data);
  }
  const uint8_t *data() const {
    return reinterpret_cast<const uint8_t *>(&actual_data[0].data);
  }

  uint32_t partial_deliveries = 0;

  // Returns a human readable peer IP address.
  std::string PeerAddress() const;

  // Prints out the RcvInfo structure.
  void LogRcvInfo() const;

  // The start of the data.
  OveralignedChar actual_data[];
};

void PrintNotification(const Message *msg);

std::string GetHostname();

// Gets and logs the contents of the sctp_status message.
void LogSctpStatus(int fd, sctp_assoc_t assoc_id);

// Manages reading and writing SCTP messages.
class SctpReadWrite {
 public:
  SctpReadWrite() = default;
  ~SctpReadWrite() { CloseSocket(); }

  // Opens a new socket.
  void OpenSocket(const struct sockaddr_storage &sockaddr_local);

  // Sends a message to the kernel.
  // Returns true for success. Will not send a partial message on failure.
  bool SendMessage(int stream, std::string_view data, int time_to_live,
                   std::optional<struct sockaddr_storage> sockaddr_remote,
                   sctp_assoc_t snd_assoc_id);

  // Reads from the kernel until a complete message is received or it blocks.
  // Returns nullptr if the kernel blocks before returning a complete message.
  aos::unique_c_ptr<Message> ReadMessage();

  int fd() const { return fd_; }

  void SetMaxSize(size_t max_size) {
    max_size_ = max_size;
    if (fd_ != -1) {
      DoSetMaxSize();
    }
  }

 private:
  void CloseSocket();
  void DoSetMaxSize();

  int fd_ = -1;

  // We use this as a unique identifier that just increments for each message.
  uint32_t send_ppid_ = 0;

  size_t max_size_ = 1000;
};

// Returns the max network buffer available for reading for a socket.
size_t ReadRMemMax();
// Returns the max network buffer available for writing for a socket.
size_t ReadWMemMax();

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_SCTP_LIB_H_
