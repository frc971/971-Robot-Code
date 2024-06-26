#include <chrono>

#include "absl/flags/flag.h"
#include "absl/log/log.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/sctp_server.h"

ABSL_FLAG(std::string, config, "aos_config.json", "Path to the config.");
ABSL_FLAG(uint32_t, port, 1323, "Port to pingpong on");
ABSL_FLAG(uint32_t, size, 1000000, "Size of data to send in bytes");
ABSL_FLAG(uint32_t, duration, 1000, "Period to send at in milliseconds");
ABSL_FLAG(uint32_t, ttl, 0, "TTL in milliseconds");

namespace aos {
namespace message_bridge {

namespace chrono = std::chrono;

class PingServer {
 public:
  PingServer(aos::ShmEventLoop *event_loop)
      : event_loop_(event_loop), server_(2, "::", absl::GetFlag(FLAGS_port)) {
    event_loop_->epoll()->OnReadable(server_.fd(),
                                     [this]() { MessageReceived(); });
    server_.SetMaxReadSize(absl::GetFlag(FLAGS_size) + 100);
    server_.SetMaxWriteSize(absl::GetFlag(FLAGS_size) + 100);

    timer_ = event_loop_->AddTimer([this]() { Timer(); });

    event_loop_->OnRun([this]() {
      timer_->Schedule(event_loop_->monotonic_now(),
                       chrono::milliseconds(absl::GetFlag(FLAGS_duration)));
    });

    event_loop_->SetRuntimeRealtimePriority(5);
  }

  ~PingServer() { event_loop_->epoll()->DeleteFd(server_.fd()); }

  aos::TimerHandler *timer_;
  sctp_assoc_t sac_assoc_id_ = 0;

  void Timer() {
    if (sac_assoc_id_ == 0) {
      LOG(INFO) << "No client, not sending";
      return;
    }

    std::string data(absl::GetFlag(FLAGS_size), 'a');

    if (server_.Send(data, sac_assoc_id_, 0, absl::GetFlag(FLAGS_ttl))) {
      LOG(INFO) << "Sent " << data.size();
    } else {
      PLOG(ERROR) << "Failed to send";
    }
  }

  void MessageReceived() {
    aos::unique_c_ptr<Message> message = server_.Read();
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
    sac_assoc_id_ = assoc_id;
    server_.SetPriorityScheduler(assoc_id);
  }
  void NodeDisconnected(sctp_assoc_t /*assoc_id*/) { sac_assoc_id_ = 0; }

  void HandleData(const Message *message) {
    VLOG(1) << "Received data of length " << message->size;

    if (VLOG_IS_ON(1)) {
      message->LogRcvInfo();
    }
  }

 private:
  aos::ShmEventLoop *event_loop_;
  SctpServer server_;
};

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());
  PingServer server(&event_loop);
  event_loop.Run();

  return EXIT_SUCCESS;
}

}  // namespace message_bridge
}  // namespace aos

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  return aos::message_bridge::Main();
}
