#include "frc971/control_loops/state_feedback_loop.h"

#include "gtest/gtest.h"

namespace testing {

// Tests that everything compiles and nothing crashes even if
// number_of_inputs!=number_of_outputs.
// There used to be lots of bugs in this area.
TEST(StateFeedbackLoopTest, UnequalSizes) {
  // In general, most (all?) errors will make these statements either not
  // compile or have assertion failures at runtime.
  const StateFeedbackPlantCoefficients<2, 4, 7> coefficients(
      Eigen::Matrix<double, 2, 2>::Identity(),
      Eigen::Matrix<double, 2, 4>::Identity(),
      Eigen::Matrix<double, 7, 2>::Identity(),
      Eigen::Matrix<double, 7, 4>::Identity(),
      Eigen::Matrix<double, 4, 1>::Constant(1),
      Eigen::Matrix<double, 4, 1>::Constant(-1));

  {
    ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 4, 7>>> v;
    v.emplace_back(new StateFeedbackPlantCoefficients<2, 4, 7>(coefficients));
    StateFeedbackPlant<2, 4, 7> plant(&v);
    plant.Update();
    plant.Reset();
    plant.CheckU();
  }
  {
    StateFeedbackLoop<2, 4, 7> test_loop(StateFeedbackController<2, 4, 7>(
        Eigen::Matrix<double, 2, 7>::Identity(),
        Eigen::Matrix<double, 4, 2>::Identity(),
        Eigen::Matrix<double, 2, 2>::Identity(), coefficients));
    test_loop.Correct(Eigen::Matrix<double, 7, 1>::Identity());
    test_loop.Update(false);
    test_loop.CapU();
  }
  {
    StateFeedbackLoop<2, 4, 7> test_loop(
        Eigen::Matrix<double, 2, 7>::Identity(),
        Eigen::Matrix<double, 4, 2>::Identity(),
        Eigen::Matrix<double, 2, 2>::Identity(), coefficients);
    test_loop.Correct(Eigen::Matrix<double, 7, 1>::Identity());
    test_loop.Update(false);
    test_loop.CapU();
  }
}

}  // namespace testing
