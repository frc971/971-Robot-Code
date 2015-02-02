#include "frc971/control_loops/team_number_test_environment.h"

#include "aos/common/network/team_number.h"

namespace frc971 {
namespace control_loops {
namespace testing {

void TeamNumberEnvironment::SetUp() {
  ::aos::network::OverrideTeamNumber(kTeamNumber);
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
