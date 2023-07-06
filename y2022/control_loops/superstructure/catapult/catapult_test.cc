#include "y2022/control_loops/superstructure/catapult/catapult.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

#include "y2022/control_loops/superstructure/catapult/catapult_plant.h"

namespace y2022 {
namespace control_loops {
namespace superstructure {
namespace catapult {
namespace testing {

// Tests that computing P and q with 2 different horizons comes out the same.
TEST(MPCTest, HorizonTest) {
  Eigen::Matrix<double, 2, 1> X_initial(0.0, 0.0);
  Eigen::Matrix<double, 2, 1> X_final(2.0, 25.0);

  CatapultProblemGenerator g(35);

  CatapultProblemGenerator g2(10);
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

  CatapultProblemGenerator g(35);
  const StateFeedbackPlant<2, 1, 1> plant = MakeCatapultPlant();

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

}  // namespace testing
}  // namespace catapult
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022
