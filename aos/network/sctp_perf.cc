#include <chrono>

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/sctp_client.h"
#include "aos/network/sctp_lib.h"
#include "aos/network/sctp_server.h"

// The casts required to read datastructures from sockets trip - Wcast - align.
#ifdef __clang
#pragma clang diagnostic ignored "-Wcast-align"
#endif

DEFINE_string(config, "aos_config.json", "Path to the config.");
DEFINE_uint32(port, 1323, "Port to run the sctp test on");
DEFINE_uint32(payload_size, 1000, "Size of data to send in bytes");
DEFINE_uint32(ttl, 0, "TTL in milliseconds");
DEFINE_uint32(rx_size, 1000000,
              "RX buffer size to set the max size to be in bytes.");
DEFINE_string(host, "", "Server host (acts as server if unspecified)");

DEFINE_bool(client, false,
            "If true, then act as a client, otherwise act as a server");
DEFINE_uint32(skip_first_n, 10,
              "Skip the first 'n' messages when computing statistics.");

DEFINE_string(sctp_auth_key_file, "",
              "When set, use the provided key for SCTP authentication as "
              "defined in RFC 4895");

DECLARE_bool(die_on_malloc);

namespace aos::message_bridge::perf {

namespace {

using util::ReadFileToVecOrDie;

SctpAuthMethod SctpAuthMethod() {
  return FLAGS_sctp_auth_key_file.empty() ? SctpAuthMethod::kNoAuth
                                          : SctpAuthMethod::kAuth;
}

std::vector<uint8_t> GetSctpAuthKey() {
  if (SctpAuthMethod() == SctpAuthMethod::kNoAuth) {
    return {};
  }
  return ReadFileToVecOrDie(FLAGS_sctp_auth_key_file);
}

}  // namespace

namespace chrono = std::chrono;

class Server {
 public:
  Server(aos::ShmEventLoop *event_loop)
      : event_loop_(event_loop),
        server_(2, "0.0.0.0", FLAGS_port, SctpAuthMethod()) {
    server_.SetAuthKey(GetSctpAuthKey());
    event_loop_->epoll()->OnReadable(server_.fd(),
                                     [this]() { MessageReceived(); });
    server_.SetMaxReadSize(FLAGS_rx_size + 100);
    server_.SetMaxWriteSize(FLAGS_rx_size + 100);

    event_loop_->SetRuntimeRealtimePriority(5);
  }

  ~Server() { event_loop_->epoll()->DeleteFd(server_.fd()); }

  void SendMessage(std::string_view message) {
    if (sac_assoc_id_ == 0) {
      LOG(INFO) << "Lost connection to client. Not sending";
      return;
    }
    if (server_.Send(message, sac_assoc_id_, 0, FLAGS_ttl)) {
      LOG(INFO) << "Server reply with " << message.size() << "B";
    } else {
      PLOG(FATAL) << "Failed to send";
    }
  }

  void MessageReceived() {
    LOG(INFO) << "Received message";
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
      SendMessage(
          std::string_view((const char *)message->data(), message->size));
    }
  }

  void NodeConnected(sctp_assoc_t assoc_id) {
    sac_assoc_id_ = assoc_id;
    server_.SetPriorityScheduler(assoc_id);
  }
  void NodeDisconnected(sctp_assoc_t /*assoc_id*/) { sac_assoc_id_ = 0; }

 private:
  sctp_assoc_t sac_assoc_id_ = 0;
  aos::ShmEventLoop *event_loop_;
  SctpServer server_;
};

class Client {
 public:
  Client(aos::ShmEventLoop *event_loop)
      : event_loop_(event_loop),
        client_(FLAGS_host, FLAGS_port, 2, "0.0.0.0", FLAGS_port,
                SctpAuthMethod()) {
    client_.SetAuthKey(GetSctpAuthKey());
    client_.SetMaxReadSize(FLAGS_rx_size + 100);
    client_.SetMaxWriteSize(FLAGS_rx_size + 100);

    timer_ = event_loop_->AddTimer([this]() { Ping(); });

    event_loop_->OnRun([this]() {
      timer_->Schedule(event_loop_->monotonic_now(),
                       chrono::milliseconds(1000));
    });

    event_loop_->epoll()->OnReadable(client_.fd(),
                                     [this]() { MessageReceived(); });
    event_loop_->SetRuntimeRealtimePriority(5);
  }

  ~Client() { event_loop_->epoll()->DeleteFd(client_.fd()); }

  void Ping() {
    std::string payload(FLAGS_payload_size, 'a');
    sent_time_ = aos::monotonic_clock::now();
    if (client_.Send(0, payload, FLAGS_ttl)) {
      LOG(INFO) << "Sending " << payload.size() << "B";
    } else {
      PLOG(ERROR) << "Failed to send";
    }
  }

  void MessageReceived() {
    aos::unique_c_ptr<Message> message = client_.Read();
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

  void HandleData(const Message *) {
    count_++;
    if (count_ <= 0) {
      LOG(INFO) << "Got message: Skipping " << -count_;
      return;
    }
    auto elapsed = aos::monotonic_clock::now() - sent_time_;
    double elapsed_secs =
        std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
            .count();
    avg_latency_ = (avg_latency_ * (count_ - 1) + elapsed_secs) / count_;
    // average one-way throughput
    double throughput = FLAGS_payload_size * 2.0 / elapsed_secs;
    double avg_throughput = FLAGS_payload_size * 2.0 / avg_latency_;
    printf(
        "Round trip: %.2fms | %.2f KB/s | Avg RTL: %.2fms | %.2f KB/s | "
        "Count: %d\n",
        elapsed_secs * 1000, throughput / 1024, avg_latency_ * 1000,
        avg_throughput / 1024, count_);
  }

 private:
  aos::ShmEventLoop *event_loop_;
  SctpClient client_;
  aos::TimerHandler *timer_;
  double avg_latency_ = 0.0;
  int count_ = -FLAGS_skip_first_n;

  aos::monotonic_clock::time_point sent_time_;
};

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());
  if (FLAGS_client) {
    CHECK(!FLAGS_host.empty()) << "Client Usage: `sctp_perf --client --host "
                                  "abc.com --payload_size [bytes] "
                                  "[--port PORT] [--config PATH]`";

    Client client(&event_loop);
    event_loop.Run();
  } else {
    CHECK(FLAGS_host.empty()) << "Server Usage: `sctp_perf [--config PATH]`";
    Server server(&event_loop);
    event_loop.Run();
  }

  return EXIT_SUCCESS;
}

}  // namespace aos::message_bridge::perf

int main(int argc, char **argv) {
  gflags::SetUsageMessage(absl::StrCat(
      "Measure SCTP performance\n", "  Server Usage: `sctp_perf`\n",
      "  Client Usage: `sctp_perf --client --host abc.com`\n"));
  aos::InitGoogle(&argc, &argv);

  // Client and server need to malloc.
  FLAGS_die_on_malloc = false;
  return aos::message_bridge::perf::Main();
}
