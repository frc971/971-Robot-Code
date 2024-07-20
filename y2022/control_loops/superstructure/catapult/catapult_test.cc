#include "frc971/control_loops/catapult/catapult.h"

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "y2022/control_loops/superstructure/catapult/catapult_plant.h"

namespace y2022::control_loops::superstructure::catapult::testing {
using CatapultProblemGenerator =
    frc971::control_loops::catapult::CatapultProblemGenerator;
using MPCProblem = frc971::control_loops::catapult::MPCProblem;
// Tests that computing P and q with 2 different horizons comes out the same.
TEST(MPCTest, HorizonTest) {
  Eigen::Matrix<double, 2, 1> X_initial(0.0, 0.0);
  Eigen::Matrix<double, 2, 1> X_final(2.0, 25.0);

  CatapultProblemGenerator g(MakeCatapultPlant(), 35);

  CatapultProblemGenerator g2(MakeCatapultPlant(), 10);
  constexpr int kTestHorizon = 10;
  EXPECT_TRUE(g2.P(kTestHorizon) == g.P(kTestHorizon))
      << g2.P(kTestHorizon) - g.P(kTestHorizon);
  EXPECT_TRUE(g2.q(kTestHorizon, X_initial, X_final) ==
              g.q(kTestHorizon, X_initial, X_final))
      << g2.q(kTestHorizon, X_initial, X_final) -
             g.q(kTestHorizon, X_initial, X_final);
}

// Tests that we can get to the destination.
TEST(MPCTest, NearGoal) {
  Eigen::Matrix<double, 2, 1> X_initial(0.0, 0.0);
  Eigen::Matrix<double, 2, 1> X_final(2.0, 25.0);

  const StateFeedbackPlant<2, 1, 1> plant = MakeCatapultPlant();
  CatapultProblemGenerator g(MakeCatapultPlant(), 35);

  std::vector<std::unique_ptr<MPCProblem>> problems;
  for (size_t i = g.horizon(); i > 0; --i) {
    problems.emplace_back(g.MakeProblem(i));
  }

  Eigen::Vector2d X = X_initial;
  for (size_t i = 0; i < g.horizon(); ++i) {
    problems[i]->SetState(X, X_final);
    if (i != 0) {
      problems[i]->WarmStart(*problems[i - 1]);
    }

    problems[i]->Solve();
    X = plant.A() * X + plant.B() * problems[i]->U(0);
  }

  EXPECT_NEAR(X_final.x(), X.x(), 1e-2);
  EXPECT_NEAR(X_final.y(), X.y(), 1e-2);
}

}  // namespace y2022::control_loops::superstructure::catapult::testing
