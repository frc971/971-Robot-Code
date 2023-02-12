#include "aos/init.h"
#include "aos/seasocks/seasocks_logger.h"
#include "glog/logging.h"
#include "internal/Embedded.h"
#include "seasocks/Server.h"

DEFINE_string(data_path, "external/foxglove_studio",
              "Path to foxglove studio files to serve.");
DEFINE_uint32(port, 8000, "Port to serve files at.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  // Magic for seasocks.
  findEmbeddedContent("");
  ::seasocks::Server server(std::make_shared<aos::seasocks::SeasocksLogger>(
      ::seasocks::Logger::Level::Info));
  server.serve(FLAGS_data_path.c_str(), FLAGS_port);
}
