#include <chrono>
#include <iostream>
#include <unordered_map>

#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "gflags/gflags.h"
#include "starter_rpc_lib.h"

DEFINE_string(config, "./config.json", "File path of aos configuration");

static const std::unordered_map<std::string, aos::starter::Command> kCommands{
    {"start", aos::starter::Command::START},
    {"stop", aos::starter::Command::STOP},
    {"restart", aos::starter::Command::RESTART}};

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::InitNRT();

  CHECK(argc == 3) << "Invalid number of command arguments";

  const std::string application_name = argv[1];
  const std::string command_str = argv[2];

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  if (command_str == "status") {
    auto status = aos::starter::GetStatus(application_name, &config.message());
    std::cout << aos::FlatbufferToJson(&status.message()) << '\n';

    return 0;
  }

  const auto command_search = kCommands.find(command_str);
  CHECK(command_search != kCommands.end())
      << "Invalid command \"" << command_str << "\"";
  const aos::starter::Command command = command_search->second;

  if (aos::starter::SendCommandBlocking(command, application_name,
                                        &config.message(),
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
    std::cout << "Failed to " << command_str << ' ' << application_name << '\n';
  }
}
