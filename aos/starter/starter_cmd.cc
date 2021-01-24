#include <chrono>
#include <functional>
#include <iostream>
#include <unordered_map>

#include "absl/strings/str_format.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"
#include "starter_rpc_lib.h"

DEFINE_string(config, "./config.json", "File path of aos configuration");

namespace {

static const std::unordered_map<std::string, aos::starter::Command>
    kCommandConversions{{"start", aos::starter::Command::START},
                        {"stop", aos::starter::Command::STOP},
                        {"restart", aos::starter::Command::RESTART}};

bool GetStarterStatus(int argc, char **argv, const aos::Configuration *config) {
  if (argc == 1) {
    // Print status for all processes.
    auto status = aos::starter::GetStarterStatus(config);
    for (const aos::starter::ApplicationStatus *app_status :
         *status.message().statuses()) {
      absl::PrintF("%-30s %s\n", app_status->name()->string_view(),
                   aos::starter::EnumNameState(app_status->state()));
    }
  } else if (argc == 2) {
    // Print status for the specified process.
    const char *application_name = argv[1];
    auto status = aos::starter::GetStatus(application_name, config);
    std::cout << aos::FlatbufferToJson(&status.message()) << '\n';
  } else {
    LOG(ERROR) << "The \"status\" command requires zero or one arguments.";
    return true;
  }
  return false;
}

bool InteractWithProgram(int argc, char **argv,
                         const aos::Configuration *config) {
  const char *command_string = argv[0];

  if (argc != 2) {
    LOG(ERROR) << "The \"" << command_string
               << "\" command requires an application name as an argument.";
    return true;
  }

  const auto command_search = kCommandConversions.find(command_string);
  CHECK(command_search != kCommandConversions.end())
      << "Internal error: \"" << command_string
      << "\" is not in kCommandConversions.";

  const aos::starter::Command command = command_search->second;
  const char *application_name = argv[1];

  if (aos::starter::SendCommandBlocking(command, application_name, config,
                                        std::chrono::seconds(3))) {
    switch (command) {
      case aos::starter::Command::START:
        std::cout << "Successfully started " << application_name << '\n';
        break;
      case aos::starter::Command::STOP:
        std::cout << "Successfully stopped " << application_name << '\n';
        break;
      case aos::starter::Command::RESTART:
        std::cout << "Successfully restarted " << application_name << '\n';
        break;
    }
  } else {
    std::cout << "Failed to " << command_string << ' ' << application_name
              << '\n';
  }
  return false;
}

// This is the set of subcommands we support. Each subcommand accepts argc and
// argv from its own point of view. So argv[0] is always the name of the
// subcommand. argv[1] and up are the arguments to the subcommand.
// The subcommand returns true if there was an error parsing the command line
// arguments. It returns false when the command line arguments are parsed
// successfully.
static const std::unordered_map<
    std::string, std::function<bool(int argc, char **argv,
                                    const aos::Configuration *config)>>
    kCommands{
        {"status", GetStarterStatus},
        {"start", InteractWithProgram},
        {"stop", InteractWithProgram},
        {"restart", InteractWithProgram},
    };

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
    auto it = kCommands.find(command);
    if (it == kCommands.end()) {
      parsing_failed = true;
    } else {
      parsing_failed = it->second(argc - 1, argv + 1, &config.message());
    }
  }

  if (parsing_failed) {
    LOG(ERROR) << "Parsing failed. Valid commands are:";
    for (auto entry: kCommands) {
      LOG(ERROR) << " - " << entry.first;
    }
    return 1;
  }

  return 0;
}
