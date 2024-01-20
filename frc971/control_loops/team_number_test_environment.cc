#include "frc971/control_loops/team_number_test_environment.h"

#include "aos/network/team_number.h"

namespace frc971::control_loops::testing {

void TeamNumberEnvironment::SetUp() {
  ::aos::network::OverrideTeamNumber(kTeamNumber);
}

}  // namespace frc971::control_loops::testing
