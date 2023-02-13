#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2023/vision/game_pieces.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");

namespace y2023 {
namespace vision {
namespace {

void GamePiecesDetectorMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);
  aos::ShmEventLoop event_loop(&config.message());
  GamePiecesDetector game_pieces_detector(&event_loop);
  event_loop.Run();
}
}  // namespace
}  // namespace vision
}  // namespace y2023

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2023::vision::GamePiecesDetectorMain();
}
