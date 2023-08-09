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

DEFINE_uint32(queue_initialization_threads, 0,
              "Number of threads to spin up to initialize the queue.  0 means "
              "use the main thread.");

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
      status_timer_(event_loop_.AddPhasedLoop(
          [this](int elapsed_cycles) {
            ServiceTimingReportFetcher(elapsed_cycles);
            SendStatus();
            status_count_ = 0;
          },
          std::chrono::milliseconds(1000))),
      cleanup_timer_(event_loop_.AddTimer([this] {
        event_loop_.Exit();
        LOG(INFO) << "Starter event loop exit finished.";
      })),
      max_status_count_(
          event_loop_.GetChannel<aos::starter::Status>("/aos")->frequency() -
          1),
      timing_report_fetcher_(
          event_loop_.MakeFetcher<aos::timing::Report>("/aos")),
      shm_base_(FLAGS_shm_base),
      listener_(&event_loop_,
                [this](signalfd_siginfo signal) { OnSignal(signal); }),
      top_(&event_loop_) {
  event_loop_.SkipAosLog();

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
    LOG(INFO) << "Starting to initialize shared memory.";
    const aos::Node *this_node = event_loop_.node();
    std::vector<const aos::Channel *> channels_to_construct;
    for (const aos::Channel *channel : *config_msg_->channels()) {
      if (aos::configuration::ChannelIsReadableOnNode(channel, this_node)) {
        if (FLAGS_queue_initialization_threads == 0) {
          AddChannel(channel);
        } else {
          channels_to_construct.push_back(channel);
        }
      }
    }

    if (FLAGS_queue_initialization_threads != 0) {
      std::mutex pool_mutex;
      std::vector<std::thread> threads;
      threads.reserve(FLAGS_queue_initialization_threads);
      for (size_t i = 0; i < FLAGS_queue_initialization_threads; ++i) {
        threads.emplace_back([this, &pool_mutex, &channels_to_construct]() {
          while (true) {
            const aos::Channel *channel;
            {
              std::unique_lock<std::mutex> locker(pool_mutex);
              if (channels_to_construct.empty()) {
                return;
              }
              channel = channels_to_construct.back();
              channels_to_construct.pop_back();
            }
            AddChannel(channel);
          }
        });
      }
      for (size_t i = 0; i < FLAGS_queue_initialization_threads; ++i) {
        threads[i].join();
      }
    }
    LOG(INFO) << "Starting applications.";
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
  cleanup_timer_->Schedule(event_loop_.monotonic_now() +
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

void Starter::ServiceTimingReportFetcher(int elapsed_cycles) {
  // If there is any chance that it has been longer than one cycle since we last
  // serviced the fetcher, call Fetch(). This reduces the chances that the
  // fetcher falls behind when the system is under heavy load. Dropping a few
  // timing report messages when the system is under stress is fine.
  if (timing_report_fetcher_.get() == nullptr || elapsed_cycles > 1) {
    timing_report_fetcher_.Fetch();
  }
  while (timing_report_fetcher_.FetchNext()) {
    for (auto &application : applications_) {
      application.second.ObserveTimingReport(
          timing_report_fetcher_.context().monotonic_event_time,
          timing_report_fetcher_.get());
    }
  }
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
  std::unique_ptr<aos::ipc_lib::MemoryMappedQueue> queue =
      std::make_unique<aos::ipc_lib::MemoryMappedQueue>(
          shm_base_, FLAGS_permissions, event_loop_.configuration(), channel);

  {
    std::unique_lock<std::mutex> locker(queue_mutex_);
    shm_queues_.emplace_back(std::move(queue));
  }
  VLOG(1) << "Created MemoryMappedQueue for "
          << aos::configuration::StrippedChannelToString(channel) << " under "
          << shm_base_;
}

}  // namespace starter
}  // namespace aos
