#include "y2023/constants/constants_generated.h"
#include "y2023/constants/constants_list_generated.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"

namespace y2023 {
void SendSimulationConstants(aos::SimulatedEventLoopFactory *factory, int team,
                             std::string constants_path) {
  for (const aos::Node *node : factory->nodes()) {
    std::unique_ptr<aos::EventLoop> event_loop =
        factory->MakeEventLoop("constants_sender", node);
    frc971::constants::ConstantSender<Constants, ConstantsList> sender(
        event_loop.get(), constants_path, team, "/constants");
  }
}
}  // namespace y2023

#endif  // Y2023_CONFIGURATION_SIMULATED_CONFIG_SENDER_H_
