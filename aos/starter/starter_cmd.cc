#include <chrono>
#include <functional>
#include <iostream>
#include <optional>
#include <string_view>
#include <unordered_map>

#include "absl/strings/str_format.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "gflags/gflags.h"
#include "starter_rpc_lib.h"

DEFINE_string(config, "./config.json", "File path of aos configuration");

namespace {

namespace chrono = std::chrono;

static const std::unordered_map<std::string, aos::starter::Command>
    kCommandConversions{{"start", aos::starter::Command::START},
                        {"stop", aos::starter::Command::STOP},
                        {"restart", aos::starter::Command::RESTART}};

const aos::Node *MaybeMyNode(const aos::Configuration *configuration) {
  if (!configuration->has_nodes()) {
    return nullptr;
  }

  return aos::configuration::GetMyNode(configuration);
}

bool ValidApplication(const aos::Configuration *config,
                      std::string_view application_name) {
  const aos::Node *node = MaybeMyNode(config);
  const aos::Application *application =
      aos::configuration::GetApplication(config, node, application_name);
  if (application == nullptr) {
    if (node) {
      std::cout << "Unknown application '" << application_name << "' on node '"
                << node->name()->string_view() << "'" << std::endl;
    } else {
      std::cout << "Unknown application '" << application_name << "'"
                << std::endl;
    }
    return false;
  }
  return true;
}

void PrintKey() {
  absl::PrintF("%-30s %-8s %-6s %-9s\n", "Name", "State", "PID", "Uptime");
}

void PrintApplicationStatus(const aos::starter::ApplicationStatus *app_status,
                            const aos::monotonic_clock::time_point &time) {
  const auto last_start_time = aos::monotonic_clock::time_point(
      chrono::nanoseconds(app_status->last_start_time()));
  const auto time_running =
      chrono::duration_cast<chrono::seconds>(time - last_start_time);
  if (app_status->state() == aos::starter::State::STOPPED) {
    absl::PrintF("%-30s %-8s\n", app_status->name()->string_view(),
                 aos::starter::EnumNameState(app_status->state()));
  } else {
    absl::PrintF("%-30s %-8s %-6d %-9ds\n", app_status->name()->string_view(),
                 aos::starter::EnumNameState(app_status->state()),
                 app_status->pid(), time_running.count());
  }
}

// Prints the status for all applications.
void GetAllStarterStatus(const aos::Configuration *config) {
    // Print status for all processes.
    const auto optional_status = aos::starter::GetStarterStatus(config);
    if (optional_status) {
      auto status = *optional_status;
      const auto time = aos::monotonic_clock::now();
      PrintKey();
      for (const aos::starter::ApplicationStatus *app_status :
           *status.message().statuses()) {
        PrintApplicationStatus(app_status, time);
      }
    } else {
      LOG(WARNING) << "No status found";
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

    if (!ValidApplication(config, application_name)) {
      return false;
    }
    auto status = aos::starter::GetStatus(application_name, config);
    PrintKey();
    PrintApplicationStatus(&status.message(), aos::monotonic_clock::now());
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
  const auto optional_status = aos::starter::GetStarterStatus(config);
  if (optional_status) {
    auto status = *optional_status;
    const aos::Node *my_node = MaybeMyNode(config);
    std::vector<std::pair<aos::starter::Command, std::string_view>> commands;

    for (const aos::Application *application : *config->applications()) {
      // Ignore any applications which aren't supposed to be started on this
      // node.
      if (!aos::configuration::ApplicationShouldStart(config, my_node,
                                                      application)) {
        continue;
      }

      const std::string_view application_name =
          application->name()->string_view();
      if (!application->autostart()) {
        const aos::starter::ApplicationStatus *application_status =
            aos::starter::FindApplicationStatus(status.message(),
                                                application_name);
        if (application_status->state() == aos::starter::State::STOPPED) {
          std::cout << "Skipping " << application_name
                    << " because it is STOPPED\n";
          continue;
        }
      }

      commands.emplace_back(command, application_name);
    }

    // Restart each running process
    if (aos::starter::SendCommandBlocking(commands, config,
                                          chrono::seconds(5))) {
      std::cout << success_text << "all \n";
    } else {
      std::cout << failure_text << "all \n";
    }
  } else {
    LOG(WARNING) << "Starter not running";
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
  if (!ValidApplication(config, application_name)) {
    return false;
  }

  if (aos::starter::SendCommandBlocking(command, application_name, config,
                                        chrono::seconds(5))) {
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

}  // namespace

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

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
