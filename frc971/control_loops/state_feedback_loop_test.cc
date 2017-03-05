#include "frc971/control_loops/state_feedback_loop.h"

#include "gtest/gtest.h"

namespace frc971 {
namespace control_loops {
namespace testing {

StateFeedbackHybridPlantCoefficients<3, 1, 1>
MakeIntegralShooterPlantCoefficients() {
  Eigen::Matrix<double, 1, 3> C;
  C(0, 0) = 1.0;
  C(0, 1) = 0.0;
  C(0, 2) = 0.0;
  Eigen::Matrix<double, 1, 1> D;
  D(0, 0) = 0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max(0, 0) = 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min(0, 0) = -12.0;
  Eigen::Matrix<double, 3, 3> A_continuous;
  A_continuous(0, 0) = 0.0;
  A_continuous(0, 1) = 1.0;
  A_continuous(0, 2) = 0.0;
  A_continuous(1, 0) = 0.0;
  A_continuous(1, 1) = -8.1021414789556374;
  A_continuous(1, 2) = 443.75;
  A_continuous(2, 0) = 0.0;
  A_continuous(2, 1) = 0.0;
  A_continuous(2, 2) = 0.0;
  Eigen::Matrix<double, 3, 1> B_continuous;
  B_continuous(0, 0) = 0.0;
  B_continuous(1, 0) = 443.75;
  B_continuous(2, 0) = 0.0;
  return StateFeedbackHybridPlantCoefficients<3, 1, 1>(
      A_continuous, B_continuous, C, D, U_max, U_min);
}

StateFeedbackControllerCoefficients<3, 1, 1>
MakeIntegralShooterControllerCoefficients() {
  Eigen::Matrix<double, 1, 3> K;
  K(0, 0) = 0.0;
  K(0, 1) = 0.027731156542808996;
  K(0, 2) = 1.0;
  Eigen::Matrix<double, 1, 3> Kff;
  Kff(0, 0) = 0.0;
  Kff(0, 1) = 0.45989503537638587;
  Kff(0, 2) = 0.0;
  return StateFeedbackControllerCoefficients<3, 1, 1>(K, Kff);
}

HybridKalmanCoefficients<3, 1, 1> MakeIntegralShooterObserverCoefficients() {
  Eigen::Matrix<double, 3, 3> Q_continuous;
  Q_continuous(0, 0) = 0.0001;
  Q_continuous(0, 1) = 0.0;
  Q_continuous(0, 2) = 0.0;
  Q_continuous(1, 0) = 0.0;
  Q_continuous(1, 1) = 4.0;
  Q_continuous(1, 2) = 0.0;
  Q_continuous(2, 0) = 0.0;
  Q_continuous(2, 1) = 0.0;
  Q_continuous(2, 2) = 0.040000000000000008;
  Eigen::Matrix<double, 1, 1> R_continuous;
  R_continuous(0, 0) = 9.9999999999999995e-07;
  Eigen::Matrix<double, 3, 3> P_steady_state;
  P_steady_state(0, 0) = 7.1645559451160497e-05;
  P_steady_state(0, 1) = 0.0031205034236441768;
  P_steady_state(0, 2) = 0.00016022137220036598;
  P_steady_state(1, 0) = 0.0031205034236441768;
  P_steady_state(1, 1) = 0.25313549121689616;
  P_steady_state(1, 2) = 0.015962850974712596;
  P_steady_state(2, 0) = 0.00016022137220036598;
  P_steady_state(2, 1) = 0.015962850974712596;
  P_steady_state(2, 2) = 0.0019821816120708254;
  return HybridKalmanCoefficients<3, 1, 1>(Q_continuous, R_continuous,
                                           P_steady_state);
}

StateFeedbackHybridPlant<3, 1, 1> MakeIntegralShooterPlant() {
  ::std::vector<
      ::std::unique_ptr<StateFeedbackHybridPlantCoefficients<3, 1, 1>>>
      plants(1);
  plants[0] = ::std::unique_ptr<StateFeedbackHybridPlantCoefficients<3, 1, 1>>(
      new StateFeedbackHybridPlantCoefficients<3, 1, 1>(
          MakeIntegralShooterPlantCoefficients()));
  return StateFeedbackHybridPlant<3, 1, 1>(&plants);
}

StateFeedbackController<3, 1, 1> MakeIntegralShooterController() {
  ::std::vector<::std::unique_ptr<StateFeedbackControllerCoefficients<3, 1, 1>>>
      controllers(1);
  controllers[0] =
      ::std::unique_ptr<StateFeedbackControllerCoefficients<3, 1, 1>>(
          new StateFeedbackControllerCoefficients<3, 1, 1>(
              MakeIntegralShooterControllerCoefficients()));
  return StateFeedbackController<3, 1, 1>(&controllers);
}

HybridKalman<3, 1, 1> MakeIntegralShooterObserver() {
  ::std::vector<::std::unique_ptr<HybridKalmanCoefficients<3, 1, 1>>> observers(
      1);
  observers[0] = ::std::unique_ptr<HybridKalmanCoefficients<3, 1, 1>>(
      new HybridKalmanCoefficients<3, 1, 1>(
          MakeIntegralShooterObserverCoefficients()));
  return HybridKalman<3, 1, 1>(&observers);
}

StateFeedbackLoop<3, 1, 1, StateFeedbackHybridPlant<3, 1, 1>,
                  HybridKalman<3, 1, 1>>
MakeIntegralShooterLoop() {
  return StateFeedbackLoop<3, 1, 1, StateFeedbackHybridPlant<3, 1, 1>,
                           HybridKalman<3, 1, 1>>(
      MakeIntegralShooterPlant(), MakeIntegralShooterController(),
      MakeIntegralShooterObserver());
}

// Tests that everything compiles and nothing crashes even if
// number_of_inputs!=number_of_outputs.
// There used to be lots of bugs in this area.
TEST(StateFeedbackLoopTest, UnequalSizes) {
  // In general, most (all?) errors will make these statements either not
  // compile or have assertion failures at runtime.
  const StateFeedbackPlantCoefficients<2, 4, 7> coefficients(
      Eigen::Matrix<double, 2, 2>::Identity(),
      Eigen::Matrix<double, 2, 2>::Identity(),
      Eigen::Matrix<double, 2, 4>::Identity(),
      Eigen::Matrix<double, 7, 2>::Identity(),
      Eigen::Matrix<double, 7, 4>::Identity(),
      Eigen::Matrix<double, 4, 1>::Constant(1),
      Eigen::Matrix<double, 4, 1>::Constant(-1));

  // Build a plant.
  ::std::vector<::std::unique_ptr<StateFeedbackPlantCoefficients<2, 4, 7>>>
      v_plant;
  v_plant.emplace_back(
      new StateFeedbackPlantCoefficients<2, 4, 7>(coefficients));
  StateFeedbackPlant<2, 4, 7> plant(&v_plant);
  plant.Update(Eigen::Matrix<double, 4, 1>::Zero());
  plant.Reset();
  plant.CheckU(Eigen::Matrix<double, 4, 1>::Zero());

  // Now build a controller.
  ::std::vector<::std::unique_ptr<StateFeedbackControllerCoefficients<2, 4, 7>>>
      v_controller;
  v_controller.emplace_back(new StateFeedbackControllerCoefficients<2, 4, 7>(
      Eigen::Matrix<double, 4, 2>::Identity(),
      Eigen::Matrix<double, 4, 2>::Identity()));
  StateFeedbackController<2, 4, 7> controller(&v_controller);

  ::std::vector<::std::unique_ptr<StateFeedbackObserverCoefficients<2, 4, 7>>>
      v_observer;
  v_observer.emplace_back(new StateFeedbackObserverCoefficients<2, 4, 7>(
      Eigen::Matrix<double, 2, 7>::Identity()));
  StateFeedbackObserver<2, 4, 7> observer(&v_observer);

  StateFeedbackLoop<2, 4, 7> test_loop(
      ::std::move(plant), ::std::move(controller), ::std::move(observer));
  test_loop.Correct(Eigen::Matrix<double, 7, 1>::Identity());
  test_loop.Update(false);
  test_loop.CapU();
}

// Tests that the continuous to discrete calculation for the kalman filter
// matches what was computed both in Python and in Matlab.
TEST(StateFeedbackLoopTest, PythonMatch) {
  auto test_loop = MakeIntegralShooterLoop();
  test_loop.Update(false, ::std::chrono::milliseconds(5));

  Eigen::Matrix<double, 3, 3> A_discrete;
  A_discrete << 1, 0.00490008, 0.00547272, 0, 0.96029888, 2.17440921, 0, 0, 1;

  Eigen::Matrix<double, 3, 1> B_discrete;
  B_discrete << 0.00547272, 2.17440921, 0;

  Eigen::Matrix<double, 3, 3> Q_discrete;
  Q_discrete << 6.62900602e-07, 4.86205253e-05, 3.66076676e-07, 4.86205253e-05,
      1.95296358e-02, 2.18908995e-04, 3.66076676e-07, 2.18908995e-04,
      2.00000000e-04;

  Eigen::Matrix<double, 1, 1> R_discrete;
  R_discrete << 0.0002;

  EXPECT_TRUE(A_discrete.isApprox(test_loop.plant().A(), 0.001));
  EXPECT_TRUE(B_discrete.isApprox(test_loop.plant().B(), 0.001));
  EXPECT_TRUE(Q_discrete.isApprox(test_loop.observer().Q(), 0.001));
  EXPECT_TRUE(R_discrete.isApprox(test_loop.observer().R(), 0.001));
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
