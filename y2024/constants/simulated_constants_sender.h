#ifndef Y2024_CONSTANTS_SIMULATED_CONFIG_SENDER_H_
#define Y2024_CONSTANTS_SIMULATED_CONFIG_SENDER_H_

#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"

namespace y2024 {
// Returns true, to allow this to be easily called in the initializer list of a
// constructor.
bool SendSimulationConstants(
    aos::SimulatedEventLoopFactory *factory, int team,
    std::string constants_path =
        aos::testing::ArtifactPath("y2024/constants/test_constants.json"));
}  // namespace y2024

#endif  // Y2024_CONSTANTS_SIMULATED_CONFIG_SENDER_H_
