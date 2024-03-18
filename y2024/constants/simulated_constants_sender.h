#ifndef Y2024_CONSTANTS_SIMULATED_CONFIG_SENDER_H_
#define Y2024_CONSTANTS_SIMULATED_CONFIG_SENDER_H_

#include <set>

#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"

namespace y2024 {
// Returns true, to allow this to be easily called in the initializer list of a
// constructor.
// If node_names is specified, we limit ourselves to sending constants on the
// specified nodes.
bool SendSimulationConstants(
    aos::SimulatedEventLoopFactory *factory, int team,
    std::string constants_path =
        aos::testing::ArtifactPath("y2024/constants/test_constants.json"),
    const std::set<std::string_view> &node_names = {});
}  // namespace y2024

#endif  // Y2024_CONSTANTS_SIMULATED_CONFIG_SENDER_H_
