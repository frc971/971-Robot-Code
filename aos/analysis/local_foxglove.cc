#include <memory>

#include "absl/flags/flag.h"

#include "aos/init.h"
#include "aos/seasocks/seasocks_logger.h"
#include "internal/Embedded.h"
#include "seasocks/Logger.h"
#include "seasocks/Server.h"

ABSL_FLAG(std::string, data_path, "external/foxglove_studio",
          "Path to foxglove studio files to serve.");
ABSL_FLAG(uint32_t, port, 8000, "Port to serve files at.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  // Magic for seasocks.
  findEmbeddedContent("");
  ::seasocks::Server server(std::make_shared<aos::seasocks::SeasocksLogger>(
      ::seasocks::Logger::Level::Info));
  server.serve(absl::GetFlag(FLAGS_data_path).c_str(),
               absl::GetFlag(FLAGS_port));
}
