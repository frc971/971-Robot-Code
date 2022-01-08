#include "starterd_lib.h"

#include <algorithm>
#include <utility>

#include "absl/strings/str_format.h"
#include "aos/json_to_flatbuffer.h"
#include "glog/logging.h"
#include "glog/stl_logging.h"

namespace aos {
namespace starter {

const aos::Channel *StatusChannelForNode(const aos::Configuration *config,
                                         const aos::Node *node) {
  return configuration::GetChannel<Status>(config, "/aos", "", node);
}
const aos::Channel *StarterRpcChannelForNode(const aos::Configuration *config,
                                             const aos::Node *node) {
  return configuration::GetChannel<StarterRpc>(config, "/aos", "", node);
}

Starter::Starter(const aos::Configuration *event_loop_config)
    : config_msg_(event_loop_config),
      event_loop_(event_loop_config),
      status_sender_(event_loop_.MakeSender<aos::starter::Status>("/aos")),
      status_timer_(event_loop_.AddTimer([this] {
        SendStatus();
        status_count_ = 0;
      })),
      cleanup_timer_(event_loop_.AddTimer([this] { event_loop_.Exit(); })),
      max_status_count_(
          event_loop_.GetChannel<aos::starter::Status>("/aos")->frequency() -
          1),
      listener_(&event_loop_,
                [this](signalfd_siginfo signal) { OnSignal(signal); }) {
  event_loop_.SkipAosLog();

  event_loop_.OnRun([this] {
    status_timer_->Setup(event_loop_.monotonic_now(),
                         std::chrono::milliseconds(1000));
  });

  if (!aos::configuration::MultiNode(config_msg_)) {
    event_loop_.MakeWatcher(
        "/aos",
        [this](const aos::starter::StarterRpc &cmd) { HandleStarterRpc(cmd); });
  } else {
    for (const aos::Node *node : aos::configuration::GetNodes(config_msg_)) {
      const Channel *channel = StarterRpcChannelForNode(config_msg_, node);
      CHECK(channel != nullptr) << ": Failed to find channel /aos for "
                                << StarterRpc::GetFullyQualifiedName() << " on "
                                << node->name()->string_view();
      if (!aos::configuration::ChannelIsReadableOnNode(channel,
                                                       event_loop_.node())) {
        LOG(INFO) << "StarterRpc channel "
                  << aos::configuration::StrippedChannelToString(channel)
                  << " is not readable on "
                  << event_loop_.node()->name()->string_view();
      } else {
        event_loop_.MakeWatcher(channel->name()->string_view(),
                                [this](const aos::starter::StarterRpc &cmd) {
                                  HandleStarterRpc(cmd);
                                });
      }
    }
  }

  if (config_msg_->has_applications()) {
    const flatbuffers::Vector<flatbuffers::Offset<aos::Application>>
        *applications = config_msg_->applications();

    if (aos::configuration::MultiNode(config_msg_)) {
      std::string_view current_node = event_loop_.node()->name()->string_view();
      for (const aos::Application *application : *applications) {
        CHECK(application->has_nodes());
        for (const flatbuffers::String *node : *application->nodes()) {
          if (node->string_view() == current_node) {
            AddApplication(application);
            break;
          }
        }
      }
    } else {
      for (const aos::Application *application : *applications) {
        AddApplication(application);
      }
    }
  }
}

void Starter::HandleStarterRpc(const StarterRpc &command) {
  if (!command.has_command() || !command.has_name() || exiting_) {
    return;
  }

  LOG(INFO) << "Received " << aos::FlatbufferToJson(&command);

  if (command.has_nodes()) {
    CHECK(aos::configuration::MultiNode(config_msg_));
    bool relevant_to_this_node = false;
    for (const flatbuffers::String *node : *command.nodes()) {
      if (node->string_view() == event_loop_.node()->name()->string_view()) {
        relevant_to_this_node = true;
      }
    }
    if (!relevant_to_this_node) {
      return;
    }
  }
  // If not populated, restart regardless of node.

  auto search = applications_.find(command.name()->str());
  if (search != applications_.end()) {
    // If an applicatione exists by the given name, dispatch the command
    search->second.HandleCommand(command.command());
  }
}

void Starter::MaybeSendStatus() {
  if (status_count_ < max_status_count_) {
    SendStatus();
    ++status_count_;
  } else {
    VLOG(1) << "That's enough " << status_count_ << " " << max_status_count_;
  }
}

void Starter::Cleanup() {
  if (exiting_) {
    return;
  }
  exiting_ = true;
  for (auto &application : applications_) {
    application.second.Terminate();
  }
  cleanup_timer_->Setup(event_loop_.monotonic_now() +
                        std::chrono::milliseconds(1500));
}

void Starter::OnSignal(signalfd_siginfo info) {
  if (info.ssi_signo == SIGCHLD) {
    // SIGCHLD messages can be collapsed if multiple are received, so all
    // applications must check their status.
    for (auto iter = applications_.begin(); iter != applications_.end();) {
      if (iter->second.MaybeHandleSignal()) {
        iter = applications_.erase(iter);
      } else {
        ++iter;
      }
    }

    if (exiting_ && applications_.empty()) {
      event_loop_.Exit();
    }
  } else {
    LOG(INFO) << "Received signal '" << strsignal(info.ssi_signo) << "'";

    if (std::find(kStarterDeath.begin(), kStarterDeath.end(), info.ssi_signo) !=
        kStarterDeath.end()) {
      LOG(WARNING) << "Starter shutting down";
      Cleanup();
    }
  }
}

Application *Starter::AddApplication(const aos::Application *application) {
  auto [iter, success] =
      applications_.try_emplace(application->name()->str(), application,
                                &event_loop_, [this]() { MaybeSendStatus(); });
  if (success) {
    // We should be catching and handling SIGCHLD correctly in the starter, so
    // don't leave in the crutch for polling for the child process status (this
    // is less about efficiency, and more about making sure bit rot doesn't
    // result in the signal handling breaking).
    iter->second.DisableChildDeathPolling();
    return &(iter->second);
  }
  return nullptr;
}

void Starter::Run() {
#ifdef AOS_ARCHITECTURE_arm_frc
  PCHECK(setuid(0) == 0) << "Failed to change user to root";
#endif

  for (auto &application : applications_) {
    if (application.second.autostart()) {
      application.second.Start();
    }
  }

  event_loop_.Run();
}

void Starter::SendStatus() {
  aos::Sender<aos::starter::Status>::Builder builder =
      status_sender_.MakeBuilder();

  std::vector<flatbuffers::Offset<aos::starter::ApplicationStatus>> statuses;

  for (auto &application : applications_) {
    statuses.push_back(application.second.PopulateStatus(builder.fbb()));
  }

  auto statuses_fbs = builder.fbb()->CreateVector(statuses);

  aos::starter::Status::Builder status_builder(*builder.fbb());
  status_builder.add_statuses(statuses_fbs);
  builder.CheckOk(builder.Send(status_builder.Finish()));
}

}  // namespace starter
}  // namespace aos
