#ifndef Y2023_CONSTANTS_SIMULATED_CONFIG_SENDER_H_
#define Y2023_CONSTANTS_SIMULATED_CONFIG_SENDER_H_

#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"

namespace y2023 {
void SendSimulationConstants(aos::SimulatedEventLoopFactory *factory, int team,
                             std::string constants_path = testing::ArtifactPath(
                                 "y2023/constants/constants.json"));
}  // namespace y2023

#endif  // Y2023_CONSTANTS_SIMULATED_CONFIG_SENDER_H_
