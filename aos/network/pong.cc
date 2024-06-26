#include <chrono>

#include "absl/flags/flag.h"
#include "absl/log/log.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/sctp_client.h"

ABSL_FLAG(std::string, config, "aos_config.json", "Path to the config.");
ABSL_FLAG(uint32_t, port, 1323, "Port to pingpong on");
ABSL_FLAG(std::string, target, "vpu0-0a", "Host to connect to");
ABSL_FLAG(uint32_t, rx_size, 1000000,
          "RX buffer size to set the max size to be in bytes.");
ABSL_FLAG(uint32_t, size, 1000, "Size of data to send in bytes");
ABSL_FLAG(uint32_t, ttl, 0, "TTL in milliseconds");

namespace aos {
namespace message_bridge {

namespace chrono = std::chrono;

class PingClient {
 public:
  PingClient(aos::ShmEventLoop *event_loop)
      : event_loop_(event_loop),
        client_(absl::GetFlag(FLAGS_target), absl::GetFlag(FLAGS_port), 2,
                "::", 0) {
    client_.SetMaxReadSize(
        std::max(absl::GetFlag(FLAGS_rx_size), absl::GetFlag(FLAGS_size)) +
        100);
    client_.SetMaxWriteSize(
        std::max(absl::GetFlag(FLAGS_rx_size), absl::GetFlag(FLAGS_size)) +
        100);

    timer_ = event_loop_->AddTimer([this]() { Timer(); });

    event_loop_->OnRun([this]() {
      timer_->Schedule(event_loop_->monotonic_now(),
                       chrono::milliseconds(1000));
    });

    event_loop_->epoll()->OnReadable(client_.fd(),
                                     [this]() { MessageReceived(); });
    event_loop_->SetRuntimeRealtimePriority(5);
  }

  ~PingClient() { event_loop_->epoll()->DeleteFd(client_.fd()); }

  aos::TimerHandler *timer_;
  sctp_assoc_t sac_assoc_id_ = 0;

  void Timer() {
    std::string data(absl::GetFlag(FLAGS_size), 'a');

    if (client_.Send(0, data, absl::GetFlag(FLAGS_ttl))) {
      LOG(INFO) << "Sent " << data.size();
    } else {
      PLOG(ERROR) << "Failed to send";
    }
  }

  void MessageReceived() {
    aos::unique_c_ptr<Message> message = client_.Read();
    ++count_;
    if (!message) {
      return;
    }

    if (message->message_type == Message::kNotification) {
      const union sctp_notification *snp =
          (const union sctp_notification *)message->data();

      if (VLOG_IS_ON(2)) {
        PrintNotification(message.get());
      }

      switch (snp->sn_header.sn_type) {
        case SCTP_ASSOC_CHANGE: {
          const struct sctp_assoc_change *sac = &snp->sn_assoc_change;
          switch (sac->sac_state) {
            case SCTP_COMM_UP:
              NodeConnected(sac->sac_assoc_id);
              VLOG(1) << "Peer connected";
              break;
            case SCTP_COMM_LOST:
            case SCTP_SHUTDOWN_COMP:
            case SCTP_CANT_STR_ASSOC:
              NodeDisconnected(sac->sac_assoc_id);
              VLOG(1) << "Disconnect";
              break;
            case SCTP_RESTART:
              LOG(FATAL) << "Never seen this before.";
              break;
          }
        } break;
      }
    } else if (message->message_type == Message::kMessage) {
      HandleData(message.get());
    }
  }

  void NodeConnected(sctp_assoc_t assoc_id) {
    client_.SetPriorityScheduler(assoc_id);
  }
  void NodeDisconnected(sctp_assoc_t /*assoc_id*/) {}

  void HandleData(const Message *message) {
    LOG(INFO) << "Received data of length " << message->size << " total "
              << size_ << " count " << count_;
    size_ += message->size;

    if (VLOG_IS_ON(1)) {
      message->LogRcvInfo();
    }
  }

 private:
  aos::ShmEventLoop *event_loop_;
  SctpClient client_;

  size_t count_ = 0;
  size_t size_ = 0;
};

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());
  PingClient server(&event_loop);
  event_loop.Run();

  return EXIT_SUCCESS;
}

}  // namespace message_bridge
}  // namespace aos

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  return aos::message_bridge::Main();
}
