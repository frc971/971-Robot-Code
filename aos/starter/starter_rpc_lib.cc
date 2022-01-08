#include "starter_rpc_lib.h"

#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/starter/starterd_lib.h"
#include "glog/logging.h"

namespace aos {
namespace starter {

namespace {
State ExpectedStateForCommand(Command command) {
  switch (command) {
    case Command::START:
    case Command::RESTART:
      return State::RUNNING;
    case Command::STOP:
      return State::STOPPED;
  }
  return State::STOPPED;
}
}  // namespace

const aos::starter::ApplicationStatus *FindApplicationStatus(
    const aos::starter::Status &status, std::string_view name) {
  if (!status.has_statuses()) {
    return nullptr;
  }

  auto statuses = status.statuses();

  auto search =
      std::find_if(statuses->begin(), statuses->end(),
                   [name](const aos::starter::ApplicationStatus *app_status) {
                     return app_status->has_name() &&
                            app_status->name()->string_view() == name;
                   });
  if (search == statuses->end()) {
    return nullptr;
  }
  return *search;
}

std::string_view FindApplication(const std::string_view &name,
                                 const aos::Configuration *config) {
  std::string_view app_name = name;
  for (const auto app : *config->applications()) {
    if (app->has_executable_name() &&
        app->executable_name()->string_view() == name) {
      app_name = app->name()->string_view();
      break;
    }
  }
  return app_name;
}

StarterClient::StarterClient(EventLoop *event_loop)
    : event_loop_(event_loop),
      timeout_timer_(event_loop_->AddTimer([this]() { Timeout(); })),
      cmd_sender_(event_loop_->MakeSender<StarterRpc>("/aos")) {
  if (configuration::MultiNode(event_loop_->configuration())) {
    for (const aos::Node *node :
         configuration::GetNodes(event_loop_->configuration())) {
      const Channel *channel =
          StatusChannelForNode(event_loop_->configuration(), node);
      CHECK(channel != nullptr) << ": Failed to find channel /aos for "
                                << Status::GetFullyQualifiedName() << " on "
                                << node->name()->string_view();
      if (!configuration::ChannelIsReadableOnNode(channel,
                                                  event_loop_->node())) {
        VLOG(1) << "Status channel "
                << configuration::StrippedChannelToString(channel)
                << " is not readable on "
                << event_loop_->node()->name()->string_view();
      } else if (!configuration::ChannelIsReadableOnNode(
                     StarterRpcChannelForNode(event_loop_->configuration(),
                                              event_loop_->node()),
                     node)) {
        // Don't attempt to construct a status fetcher if the other node won't
        // even be able to receive our commands.
        VLOG(1) << "StarterRpc channel for "
                << event_loop_->node()->name()->string_view()
                << " is not readable on " << node->name()->string_view();
      } else {
        status_fetchers_[node->name()->str()] =
            event_loop_->MakeFetcher<Status>(channel->name()->string_view());
        event_loop_->MakeNoArgWatcher<Status>(channel->name()->string_view(),
                                              [this]() {
                                                if (CheckCommandsSucceeded()) {
                                                  Succeed();
                                                }
                                              });
      }
    }
  } else {
    status_fetchers_[""] = event_loop_->MakeFetcher<Status>("/aos");
    event_loop_->MakeNoArgWatcher<Status>("/aos", [this]() {
      if (CheckCommandsSucceeded()) {
        Succeed();
      }
    });
  }
}

void StarterClient::SendCommands(
    const std::vector<ApplicationCommand> &commands,
    monotonic_clock::duration timeout) {
  CHECK(current_commands_.empty());
  for (auto &pair : status_fetchers_) {
    pair.second.Fetch();
  }
  const bool is_multi_node =
      aos::configuration::MultiNode(event_loop_->configuration());
  for (const auto &command : commands) {
    auto builder = cmd_sender_.MakeBuilder();
    const auto application_offset =
        builder.fbb()->CreateString(command.application);
    std::vector<flatbuffers::Offset<flatbuffers::String>> node_offsets;
    CHECK(!command.nodes.empty())
        << "At least one node must be specified for application "
        << command.application;
    for (const aos::Node *node : command.nodes) {
      const std::string node_name((node == nullptr) ? "" : node->name()->str());
      if (status_fetchers_.count(node_name) == 0) {
        if (is_multi_node) {
          LOG(FATAL) << "Node \"" << node_name
                     << "\" must be configured to both receive StarterRpc "
                        "messages from \""
                     << event_loop_->node()->name()->string_view()
                     << "\" as well as to send starter Status messages back.";
        } else {
          LOG(FATAL) << "On single-node configs, use an empty string for the "
                        "node name.";
        }
      }
      CHECK(status_fetchers_[node_name].get() != nullptr)
          << ": No status available for node " << node_name;
      if (is_multi_node) {
        node_offsets.push_back(builder.fbb()->CreateString(node_name));
      }
      const ApplicationStatus *last_status =
          CHECK_NOTNULL(FindApplicationStatus(*status_fetchers_[node_name],
                                              command.application));
      current_commands_[node_name].push_back(CommandStatus{
          .expected_state = ExpectedStateForCommand(command.command),
          .application = std::string(command.application),
          .old_id = std::nullopt});
      // If we are restarting, then we need to track what the current ID of the
      // process is to ensure that it actually got restarted. For just starting,
      // we leave the application running and so don't care.
      if (command.command == Command::RESTART && last_status->has_id()) {
        current_commands_[node_name].back().old_id = last_status->id();
      }
    }
    flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
        nodes_offset;
    if (is_multi_node) {
      nodes_offset = builder.fbb()->CreateVector(node_offsets);
    }
    auto command_builder = builder.MakeBuilder<StarterRpc>();
    command_builder.add_command(command.command);
    command_builder.add_name(application_offset);
    if (is_multi_node) {
      command_builder.add_nodes(nodes_offset);
    }
    builder.CheckOk(builder.Send(command_builder.Finish()));
  }

  timeout_timer_->Setup(event_loop_->monotonic_now() + timeout);
}

bool StarterClient::CheckCommandsSucceeded() {
  if (current_commands_.empty()) {
    return false;
  }

  for (auto &pair : status_fetchers_) {
    pair.second.Fetch();
  }

  bool succeeded = true;

  for (const auto &pair : current_commands_) {
    if (pair.second.empty()) {
      continue;
    }
    CHECK(status_fetchers_[pair.first].get() != nullptr)
        << ": No status available for node " << pair.first;
    const Status &status = *status_fetchers_[pair.first];
    for (const auto &command : pair.second) {
      const ApplicationStatus *application_status =
          CHECK_NOTNULL(FindApplicationStatus(status, command.application));
      if (application_status->state() == command.expected_state) {
        if (command.expected_state == State::RUNNING &&
            application_status->id() == command.old_id) {
          succeeded = false;
        }
      } else {
        succeeded = false;
      }
    }
  }
  return succeeded;
}

void StarterClient::Timeout() {
  // Clear commands prior to calling handlers to allow the handler to call
  // SendCommands() again if desired.
  current_commands_.clear();
  if (timeout_handler_) {
    timeout_handler_();
  }
}

void StarterClient::Succeed() {
  // Clear commands prior to calling handlers to allow the handler to call
  // SendCommands() again if desired.
  current_commands_.clear();
  if (success_handler_) {
    success_handler_();
  }
  timeout_timer_->Disable();
}

bool SendCommandBlocking(aos::starter::Command command, std::string_view name,
                         const aos::Configuration *config,
                         std::chrono::milliseconds timeout,
                         std::vector<const aos::Node *> nodes) {
  return SendCommandBlocking({{command, name, nodes}}, config, timeout);
}

bool SendCommandBlocking(const std::vector<ApplicationCommand> &commands,
                         const aos::Configuration *config,
                         std::chrono::milliseconds timeout) {
  aos::ShmEventLoop event_loop(config);
  event_loop.SkipAosLog();

  StarterClient client(&event_loop);

  // Wait until event loop starts to send all commands so the watcher is ready
  event_loop.OnRun([&commands, &client, timeout]() {
    client.SendCommands(commands, timeout);
  });

  // If still waiting after timeout milliseconds, exit the loop
  client.SetTimeoutHandler([&event_loop]() { event_loop.Exit(); });

  bool success = false;

  client.SetSuccessHandler([&event_loop, &success]() {
    success = true;
    event_loop.Exit();
  });

  event_loop.Run();

  return success;
}

const std::optional<FlatbufferDetachedBuffer<aos::starter::ApplicationStatus>>
GetStatus(std::string_view name, const Configuration *config,
          const aos::Node *node) {
  ShmEventLoop event_loop(config);
  event_loop.SkipAosLog();

  auto status_fetcher = event_loop.MakeFetcher<aos::starter::Status>(
      StatusChannelForNode(config, node)->name()->string_view());
  status_fetcher.Fetch();
  if (status_fetcher.get() != nullptr) {
    const aos::starter::ApplicationStatus *status =
        FindApplicationStatus(*status_fetcher, name);
    if (status != nullptr) {
      return aos::CopyFlatBuffer(status);
    }
  }
  return std::nullopt;
}

std::optional<std::pair<aos::monotonic_clock::time_point,
                        const aos::FlatbufferVector<aos::starter::Status>>>
GetStarterStatus(const aos::Configuration *config, const aos::Node *node) {
  ShmEventLoop event_loop(config);
  event_loop.SkipAosLog();

  auto status_fetcher = event_loop.MakeFetcher<aos::starter::Status>(
      StatusChannelForNode(config, node)->name()->string_view());
  status_fetcher.Fetch();
  return (status_fetcher.get() == nullptr)
             ? std::nullopt
             : std::make_optional(std::make_pair(
                   status_fetcher.context().monotonic_remote_time,
                   status_fetcher.CopyFlatBuffer()));
}

}  // namespace starter
}  // namespace aos
