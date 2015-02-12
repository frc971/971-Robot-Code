#ifndef FRC971_CONTROL_LOOPS_TEAM_NUMBER_TEST_ENVIRONMENT_H_
#define FRC971_CONTROL_LOOPS_TEAM_NUMBER_TEST_ENVIRONMENT_H_

#include "gtest/gtest.h"

namespace frc971 {
namespace control_loops {
namespace testing {

// The team number we use for tests.
static const int kTeamNumber = 1;

// Overrides the team number to kTeamNumber before any test consructors run.
// This is important for tests which retrieve constants values during
// construction.
class TeamNumberEnvironment : public ::testing::Environment {
 public:
  void SetUp() override;
};

// The static variable in a header is intentional. Kind of a hack, undefined
// order, but that works OK here.
static ::testing::Environment* const team_number_env =
    ::testing::AddGlobalTestEnvironment(new TeamNumberEnvironment());

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_TEAM_NUMBER_TEST_ENVIRONMENT_H_
