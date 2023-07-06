#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"
#include "frc971/constants/constants_sender_lib.h"
#include "y2023/constants/constants_generated.h"
#include "y2023/constants/constants_list_generated.h"

namespace y2023 {
bool SendSimulationConstants(aos::SimulatedEventLoopFactory *factory, int team,
                             std::string constants_path) {
  for (const aos::Node *node : factory->nodes()) {
    std::unique_ptr<aos::EventLoop> event_loop =
        factory->MakeEventLoop("constants_sender", node);
    frc971::constants::ConstantSender<Constants, ConstantsList> sender(
        event_loop.get(), constants_path, team, "/constants");
  }
  return true;
}
}  // namespace y2023
