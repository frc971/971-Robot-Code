#include <chrono>
#include <functional>
#include <iostream>
#include <optional>
#include <string_view>
#include <unordered_map>

#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "gflags/gflags.h"
#include "starter_rpc_lib.h"

DEFINE_string(config, "./aos_config.json", "File path of aos configuration");
// TODO(james): Bash autocompletion for node names.
DEFINE_string(
    node, "",
    "Node to interact with. If empty, just interact with local node.");
DEFINE_bool(all_nodes, false, "Interact with all nodes.");

DEFINE_bool(_bash_autocomplete, false,
            "Internal use: Outputs commands or applications for use with "
            "autocomplete script.");
DEFINE_string(_bash_autocomplete_word, "",
              "Internal use: Current word being autocompleted");

namespace {

namespace chrono = std::chrono;

static const std::unordered_map<std::string, aos::starter::Command>
    kCommandConversions{{"start", aos::starter::Command::START},
                        {"stop", aos::starter::Command::STOP},
                        {"restart", aos::starter::Command::RESTART}};

std::vector<const aos::Node *> InteractNodes(
    const aos::Configuration *configuration) {
  if (!configuration->has_nodes()) {
    return {nullptr};
  }

  if (!FLAGS_node.empty()) {
    CHECK(!FLAGS_all_nodes) << "Can't specify both --node and --all_nodes.";
    return {aos::configuration::GetNode(configuration, FLAGS_node)};
  }

  if (FLAGS_all_nodes) {
    return aos::configuration::GetNodes(configuration);
  }

  return {aos::configuration::GetMyNode(configuration)};
}

std::vector<const aos::Node *> InteractNodesForApplication(
    const aos::Configuration *config, std::string_view application_name) {
  const std::vector<const aos::Node *> interact_nodes = InteractNodes(config);
  std::vector<const aos::Node *> application_nodes;
  std::vector<std::string> debug_node_names;
  for (const aos::Node *node : interact_nodes) {
    if (aos::configuration::GetApplication(config, node, application_name) !=
        nullptr) {
      application_nodes.push_back(node);
    }
    if (node != nullptr) {
      debug_node_names.push_back(node->name()->str());
    }
  }

  if (application_nodes.empty()) {
    if (interact_nodes.size() == 1 && interact_nodes[0] == nullptr) {
      std::cout << "Unknown application " << application_name << std::endl;
    } else {
      std::cout << "Unknown application " << application_name
                << " on any of node(s) "
                << absl::StrJoin(debug_node_names, ", ") << std::endl;
    }
  }
  return application_nodes;
}

void PrintKey() {
  absl::PrintF("%-30s %-10s %-8s %-6s %-9s\n", "Name", "Node", "State", "PID",
               "Uptime");
}

void PrintApplicationStatus(const aos::starter::ApplicationStatus *app_status,
                            const aos::monotonic_clock::time_point &time,
                            const aos::Node *node) {
  const auto last_start_time = aos::monotonic_clock::time_point(
      chrono::nanoseconds(app_status->last_start_time()));
  const auto time_running =
      chrono::duration_cast<chrono::seconds>(time - last_start_time);
  if (app_status->state() == aos::starter::State::STOPPED) {
    absl::PrintF("%-30s %-10s %-8s\n", app_status->name()->string_view(),
                 (node == nullptr) ? "none" : node->name()->string_view(),
                 aos::starter::EnumNameState(app_status->state()));
  } else {
    absl::PrintF("%-30s %-10s %-8s %-6d %-9s\n",
                 app_status->name()->string_view(),
                 (node == nullptr) ? "none" : node->name()->string_view(),
                 aos::starter::EnumNameState(app_status->state()),
                 app_status->pid(), std::to_string(time_running.count()) + 's');
  }
}

// Prints the status for all applications.
void GetAllStarterStatus(const aos::Configuration *config) {
  PrintKey();
  std::vector<const aos::Node *> missing_nodes;
  for (const aos::Node *node : InteractNodes(config)) {
    // Print status for all processes.
    const auto optional_status = aos::starter::GetStarterStatus(config, node);
    if (optional_status) {
      const aos::FlatbufferVector<aos::starter::Status> &status =
          optional_status->second;
      const aos::monotonic_clock::time_point time = optional_status->first;
      for (const aos::starter::ApplicationStatus *app_status :
           *status.message().statuses()) {
        PrintApplicationStatus(app_status, time, node);
      }
    } else {
      missing_nodes.push_back(node);
    }
  }
  for (const aos::Node *node : missing_nodes) {
    if (node == nullptr) {
      LOG(WARNING) << "No status found.";
    } else {
      LOG(WARNING) << "No status found for node "
                   << node->name()->string_view();
    }
  }
}

// Handles the "status" command.  Returns true if the help message should be
// printed.
bool GetStarterStatus(int argc, char **argv, const aos::Configuration *config) {
  if (argc == 1) {
    GetAllStarterStatus(config);
  } else if (argc == 2) {
    // Print status for the specified process.
    const auto application_name =
        aos::starter::FindApplication(argv[1], config);
    if (application_name == "all") {
      GetAllStarterStatus(config);
      return false;
    }

    const std::vector<const aos::Node *> application_nodes =
        InteractNodesForApplication(config, application_name);
    if (application_nodes.empty()) {
      return false;
    }
    PrintKey();
    for (const aos::Node *node : application_nodes) {
      auto optional_status =
          aos::starter::GetStatus(application_name, config, node);
      if (optional_status.has_value()) {
        PrintApplicationStatus(&optional_status.value().second.message(),
                               optional_status.value().first, node);
      } else {
        if (node != nullptr) {
          LOG(ERROR) << "No status available yet for \"" << application_name
                     << "\" on node \"" << node->name()->string_view() << "\".";
        } else {
          LOG(ERROR) << "No status available yet for \"" << application_name
                     << "\".";
        }
      }
    }
  } else {
    LOG(ERROR) << "The \"status\" command requires zero or one arguments.";
    return true;
  }
  return false;
}

// Sends the provided command to all applications.  Prints the success text on
// success, and failure text on failure.
void InteractWithAll(const aos::Configuration *config,
                     const aos::starter::Command command,
                     std::string_view success_text,
                     std::string_view failure_text) {
  std::map<const aos::Node *,
           std::unique_ptr<aos::FlatbufferVector<aos::starter::Status>>>
      statuses;

  for (const aos::Node *node : InteractNodes(config)) {
    std::optional<std::pair<aos::monotonic_clock::time_point,
                            const aos::FlatbufferVector<aos::starter::Status>>>
        optional_status = aos::starter::GetStarterStatus(config, node);
    if (optional_status.has_value()) {
      statuses[node] =
          std::make_unique<aos::FlatbufferVector<aos::starter::Status>>(
              optional_status.value().second);
    } else {
      if (node == nullptr) {
        LOG(WARNING) << "Starter not running";
      } else {
        LOG(WARNING) << "Starter not running on node "
                     << node->name()->string_view();
      }
    }
  }

  if (!statuses.empty()) {
    std::vector<aos::starter::ApplicationCommand> commands;

    for (const aos::Application *application : *config->applications()) {
      const std::string_view application_name =
          application->name()->string_view();
      const std::vector<const aos::Node *> application_nodes =
          InteractNodesForApplication(config, application_name);
      // Ignore any applications which aren't supposed to be started.
      if (application_nodes.empty()) {
        continue;
      }

      std::vector<const aos::Node *> running_nodes;
      if (application->autostart()) {
        running_nodes = application_nodes;
      } else {
        for (const aos::Node *node : application_nodes) {
          const aos::starter::ApplicationStatus *application_status =
              aos::starter::FindApplicationStatus(statuses[node]->message(),
                                                  application_name);
          if (application_status->state() == aos::starter::State::STOPPED) {
            if (node == nullptr) {
              std::cout << "Skipping " << application_name
                        << " because it is STOPPED\n";
            } else {
              std::cout << "Skipping " << application_name << " on "
                        << node->name()->string_view()
                        << " because it is STOPPED\n";
            }
            continue;
          } else {
            running_nodes.push_back(node);
          }
        }
      }

      if (!running_nodes.empty()) {
        commands.emplace_back(aos::starter::ApplicationCommand{
            command, application_name, running_nodes});
      }
    }

    // Restart each running process
    if (aos::starter::SendCommandBlocking(commands, config,
                                          chrono::seconds(5))) {
      std::cout << success_text << "all \n";
    } else {
      std::cout << failure_text << "all \n";
    }
  } else {
    LOG(WARNING) << "None of the starters we care about are running.";
  }
}

// Handles the "start", "stop", and "restart" commands.  Returns true if the
// help message should be printed.
bool InteractWithProgram(int argc, char **argv,
                         const aos::Configuration *config) {
  const char *command_string = argv[0];
  if (argc != 2) {
    LOG(ERROR)
        << "The \"" << command_string
        << "\" command requires an application name or 'all' as an argument.";
    return true;
  }

  const auto command_search = kCommandConversions.find(command_string);
  CHECK(command_search != kCommandConversions.end())
      << "Internal error: \"" << command_string
      << "\" is not in kCommandConversions.";
  const aos::starter::Command command = command_search->second;

  std::string_view success_text;
  const std::string failure_text =
      std::string("Failed to ") + std::string(command_string) + " ";
  switch (command) {
    case aos::starter::Command::START:
      success_text = "Successfully started ";
      break;
    case aos::starter::Command::STOP:
      success_text = "Successfully stopped ";
      break;
    case aos::starter::Command::RESTART:
      success_text = "Successfully restarted ";
      break;
  }

  const std::string_view application_name =
      aos::starter::FindApplication(argv[1], config);
  if (application_name == "all") {
    InteractWithAll(config, command, success_text, failure_text);
    return false;
  }

  const std::vector<const aos::Node *> application_nodes =
      InteractNodesForApplication(config, application_name);
  if (application_nodes.empty()) {
    return false;
  }

  if (aos::starter::SendCommandBlocking(command, application_name, config,
                                        chrono::seconds(5),
                                        application_nodes)) {
    std::cout << success_text << application_name << '\n';
  } else {
    std::cout << failure_text << application_name << '\n';
  }
  return false;
}

bool Help(int /*argc*/, char ** /*argv*/,
          const aos::Configuration * /*config*/);

// This is the set of subcommands we support. Each subcommand accepts argc and
// argv from its own point of view. So argv[0] is always the name of the
// subcommand. argv[1] and up are the arguments to the subcommand.
// The subcommand returns true if there was an error parsing the command line
// arguments. It returns false when the command line arguments are parsed
// successfully.
static const std::vector<
    std::tuple<std::string,
               std::function<bool(int argc, char **argv,
                                  const aos::Configuration *config)>,
               std::string_view>>
    kCommands{
        {"help", Help, ""},
        {"status", GetStarterStatus,
         " [application], Returns the status of the provided application, "
         "or all applications by default"},
        {"start", InteractWithProgram,
         " application, Starts the provided application, "
         "or all applications if all is provided"},
        {"stop", InteractWithProgram,
         " application, Stops the provided application, "
         "or all applications if all is provided"},
        {"restart", InteractWithProgram,
         " application, Restarts the provided application, "
         "or all applications if all is provided"}};

bool Help(int /*argc*/, char ** /*argv*/,
          const aos::Configuration * /*config*/) {
  std::cout << "Valid commands are:" << std::endl;
  for (auto entry : kCommands) {
    std::cout << " - " << std::get<0>(entry) << std::get<2>(entry) << std::endl;
  }
  return false;
}

void Autocomplete(int argc, char **argv, const aos::Configuration *config) {
  const std::string_view command = (argc >= 2 ? argv[1] : "");
  const std::string_view app_name = (argc >= 3 ? argv[2] : "");

  std::cout << "COMPREPLY=(";
  if (FLAGS__bash_autocomplete_word == command) {
    // Autocomplete the starter command
    for (const auto &entry : kCommands) {
      if (std::get<0>(entry).find(command) == 0) {
        std::cout << '\'' << std::get<0>(entry) << "' ";
      }
    }
  } else {
    // Autocomplete the app name
    for (const auto *app : *config->applications()) {
      if (app->has_name() && app->name()->string_view().find(app_name) == 0) {
        std::cout << '\'' << app->name()->string_view() << "' ";
      }
    }

    // Autocomplete with "all"
    if (std::string_view("all").find(app_name) == 0) {
      std::cout << "'all'";
    }
  }
  std::cout << ')';
}

}  // namespace

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  if (FLAGS__bash_autocomplete) {
    Autocomplete(argc, argv, &config.message());
    return 0;
  }

  bool parsing_failed = false;

  if (argc < 2) {
    parsing_failed = true;
  } else {
    const char *command = argv[1];
    auto it = std::find_if(
        kCommands.begin(), kCommands.end(),
        [command](const std::tuple<
                  std::string,
                  std::function<bool(int argc, char **argv,
                                     const aos::Configuration *config)>,
                  std::string_view> &t) { return std::get<0>(t) == command; });

    if (it == kCommands.end()) {
      parsing_failed = true;
    } else {
      parsing_failed = std::get<1>(*it)(argc - 1, argv + 1, &config.message());
    }
  }

  if (parsing_failed) {
    Help(argc - 1, argv + 1, &config.message());
    return 1;
  }

  return 0;
}
