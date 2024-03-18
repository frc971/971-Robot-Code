#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"
#include "frc971/constants/constants_sender_lib.h"
#include "y2024/constants/constants_generated.h"
#include "y2024/constants/constants_list_generated.h"

namespace y2024 {
bool SendSimulationConstants(aos::SimulatedEventLoopFactory *factory, int team,
                             std::string constants_path,
                             const std::set<std::string_view> &node_names) {
  for (const aos::Node *node : factory->nodes()) {
    if (!node_names.empty() &&
        !node_names.contains(node->name()->string_view())) {
      continue;
    }
    std::unique_ptr<aos::EventLoop> event_loop =
        factory->MakeEventLoop("constants_sender", node);
    frc971::constants::ConstantSender<Constants, ConstantsList> sender(
        event_loop.get(), constants_path, team, "/constants");
  }
  return true;
}
}  // namespace y2024
