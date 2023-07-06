#include "aos/starter/starterd_lib.h"

#include <algorithm>
#include <utility>

#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "glog/stl_logging.h"

#include "aos/json_to_flatbuffer.h"

// FLAGS_shm_base is defined elsewhere, declare it here so it can be used
// to override the shared memory folder for unit testing.
DECLARE_string(shm_base);
// FLAGS_permissions is defined elsewhere, declare it here so it can be used
// to set the file permissions on the shared memory block.
DECLARE_uint32(permissions);

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
      cleanup_timer_(event_loop_.AddTimer([this] {
        event_loop_.Exit();
        LOG(INFO) << "Starter event loop exit finished.";
      })),
      max_status_count_(
          event_loop_.GetChannel<aos::starter::Status>("/aos")->frequency() -
          1),
      shm_base_(FLAGS_shm_base),
      listener_(&event_loop_,
                [this](signalfd_siginfo signal) { OnSignal(signal); }),
      top_(&event_loop_) {
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

  // Catalogue all the applications for this node, so we can keep an eye on
  // them.
  if (config_msg_->has_applications()) {
    const flatbuffers::Vector<flatbuffers::Offset<aos::Application>>
        *applications = config_msg_->applications();

    if (aos::configuration::MultiNode(config_msg_)) {
      std::string_view current_node = event_loop_.node()->name()->string_view();
      for (const aos::Application *application : *applications) {
        CHECK(application->has_nodes())
            << ": Missing nodes on " << aos::FlatbufferToJson(application);
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

  // Catalogue all the intranode channels for this node, and create
  // MemoryMappedQueues for each one to allocate the shared memory before
  // spawning any shasta process.
  if (config_msg_->has_channels()) {
    const aos::Node *this_node = event_loop_.node();
    std::vector<const aos::Channel *> intranode_channels;
    for (const aos::Channel *channel : *config_msg_->channels()) {
      if (aos::configuration::ChannelIsReadableOnNode(channel, this_node)) {
        AddChannel(channel);
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

void Starter::HandleStateChange() {
  std::set<pid_t> all_pids;
  for (const auto &pair : applications_) {
    if (pair.second.get_pid() > 0 &&
        pair.second.status() != aos::starter::State::STOPPED) {
      all_pids.insert(pair.second.get_pid());
    }
  }
  top_.set_track_pids(all_pids);

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
  auto [iter, success] = applications_.try_emplace(
      application->name()->str(), application, &event_loop_,
      [this]() { HandleStateChange(); });
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
    statuses.push_back(application.second.PopulateStatus(builder.fbb(), &top_));
  }

  auto statuses_fbs = builder.fbb()->CreateVector(statuses);

  aos::starter::Status::Builder status_builder(*builder.fbb());
  status_builder.add_statuses(statuses_fbs);
  builder.CheckOk(builder.Send(status_builder.Finish()));
}

void Starter::AddChannel(const aos::Channel *channel) {
  CHECK_NOTNULL(channel);
  shm_queues_.emplace_back(std::make_unique<aos::ipc_lib::MemoryMappedQueue>(
      shm_base_, FLAGS_permissions, event_loop_.configuration(), channel));
  VLOG(1) << "Created MemoryMappedQueue for "
          << aos::configuration::StrippedChannelToString(channel) << " under "
          << shm_base_;
}

}  // namespace starter
}  // namespace aos
