#include "aos/network/message_bridge_auth_client_lib.h"

#include <vector>

#include "glog/logging.h"

#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_auth.grpc.pb.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/sctp_config_generated.h"
#include "aos/network/sctp_config_request_generated.h"
#include "aos/util/file.h"
#include "grpc/grpc.h"
#include "grpcpp/channel.h"

namespace aos::message_bridge::auth {

namespace {
using ::grpc::Channel;
using ::grpc::ClientContext;
using ::grpc::Status;
}  // namespace

MessageBridgeAuthClient::MessageBridgeAuthClient(
    EventLoop *const event_loop, std::shared_ptr<Channel> channel)
    : event_loop_(event_loop),
      sender_(event_loop_->MakeSender<SctpConfig>("/aos")),
      poll_timer_(event_loop_->AddTimer([this] {
        // We use a polling model as opposed to a watcher. The reason is that
        // if the watcher is not fast enough to handle the messages, we may
        // fill up the channel and crash. A polling model doesn't have that
        // problem and we don't generally care if we miss a message as we will
        // get another request in the future anyway.
        if (config_request_fetcher_.Fetch()) {
          if (config_request_fetcher_->request_key()) {
            VLOG(1) << "Got SCTP authentication request from /aos";
            SendKey();
          }
        }
      })),
      config_request_fetcher_(
          event_loop_->MakeFetcher<SctpConfigRequest>("/aos")),
      client_(SctpConfigService::NewStub(channel)) {
  event_loop_->OnRun([this] {
    poll_timer_->Schedule(event_loop_->monotonic_now(),
                          std::chrono::milliseconds(1000));
  });
}

void MessageBridgeAuthClient::SendKey() {
  std::vector<uint8_t> key(GetSctpKey());
  if (key.empty()) {
    return;
  }
  Sender<SctpConfig>::Builder sender = sender_.MakeBuilder();
  auto fb_key = sender.fbb()->CreateVector(std::move(key));
  auto builder = sender.MakeBuilder<SctpConfig>();
  builder.add_key(fb_key);
  sender.CheckOk(sender.Send(builder.Finish()));
}

std::vector<uint8_t> MessageBridgeAuthClient::GetSctpKey() {
  ClientContext context;
  context.set_deadline(std::chrono::system_clock::now() +
                       std::chrono::seconds(1));
  SctpKeyRequest request;
  SctpKeyResponse response;
  Status status = client_->GetActiveKey(&context, request, &response);
  if (!status.ok()) {
    LOG_EVERY_N(ERROR, 50)
        << "Unable to retreive active SCTP authentication key from server";
    return {};
  }

  return std::vector<uint8_t>(response.key().begin(), response.key().end());
}
}  // namespace aos::message_bridge::auth
