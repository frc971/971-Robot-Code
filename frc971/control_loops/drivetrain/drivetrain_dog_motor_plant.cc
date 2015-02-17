#include "frc971/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00489267131849, 0.0, 1.91843505092e-05, 0.0, 0.957387962253, 0.0, 0.00756271754847, 0.0, 1.91843505092e-05, 1.0, 0.00489267131849, 0.0, 0.00756271754847, 0.0, 0.957387962253;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.95240483493e-05, -7.06468379563e-06, 0.0156919866758, -0.0027849891551, -7.06468379563e-06, 3.95240483493e-05, -0.0027849891551, 0.0156919866758;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowHighPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00489267131849, 0.0, 1.91843505092e-05, 0.0, 0.957387962253, 0.0, 0.00756271754847, 0.0, 1.91843505092e-05, 1.0, 0.00489267131849, 0.0, 0.00756271754847, 0.0, 0.957387962253;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.95240483493e-05, -7.06468379563e-06, 0.0156919866758, -0.0027849891551, -7.06468379563e-06, 3.95240483493e-05, -0.0027849891551, 0.0156919866758;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainHighLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00489267131849, 0.0, 1.91843505092e-05, 0.0, 0.957387962253, 0.0, 0.00756271754847, 0.0, 1.91843505092e-05, 1.0, 0.00489267131849, 0.0, 0.00756271754847, 0.0, 0.957387962253;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.95240483493e-05, -7.06468379563e-06, 0.0156919866758, -0.0027849891551, -7.06468379563e-06, 3.95240483493e-05, -0.0027849891551, 0.0156919866758;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainHighHighPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00489267131849, 0.0, 1.91843505092e-05, 0.0, 0.957387962253, 0.0, 0.00756271754847, 0.0, 1.91843505092e-05, 1.0, 0.00489267131849, 0.0, 0.00756271754847, 0.0, 0.957387962253;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.95240483493e-05, -7.06468379563e-06, 0.0156919866758, -0.0027849891551, -7.06468379563e-06, 3.95240483493e-05, -0.0027849891551, 0.0156919866758;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<4, 2, 2> MakeDrivetrainLowLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.25738796225, 0.00756271754847, 74.8971101634, 1.58403340092, 0.00756271754847, 1.25738796225, 1.58403340092, 74.8971101634;
  Eigen::Matrix<double, 2, 4> K;
  K << 154.070413936, 12.2263299292, 2.23580489275, 0.77037863276, 2.23580489275, 0.77037863276, 154.070413936, 12.2263299292;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00511059808244, 0.0, 2.03320493463e-05, 0.0, 1.04457382236, 0.0, -0.00825142689119, 0.0, 2.03320493463e-05, 1.0, -0.00511059808244, 0.0, -0.00825142689119, 0.0, 1.04457382236;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainLowHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.25738796225, 0.00756271754847, 74.8971101634, 1.58403340092, 0.00756271754847, 1.25738796225, 1.58403340092, 74.8971101634;
  Eigen::Matrix<double, 2, 4> K;
  K << 154.070413936, 12.2263299292, 2.23580489275, 0.77037863276, 2.23580489275, 0.77037863276, 154.070413936, 12.2263299292;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00511059808244, 0.0, 2.03320493463e-05, 0.0, 1.04457382236, 0.0, -0.00825142689119, 0.0, 2.03320493463e-05, 1.0, -0.00511059808244, 0.0, -0.00825142689119, 0.0, 1.04457382236;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.25738796225, 0.00756271754847, 74.8971101634, 1.58403340092, 0.00756271754847, 1.25738796225, 1.58403340092, 74.8971101634;
  Eigen::Matrix<double, 2, 4> K;
  K << 154.070413936, 12.2263299292, 2.23580489275, 0.77037863276, 2.23580489275, 0.77037863276, 154.070413936, 12.2263299292;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00511059808244, 0.0, 2.03320493463e-05, 0.0, 1.04457382236, 0.0, -0.00825142689119, 0.0, 2.03320493463e-05, 1.0, -0.00511059808244, 0.0, -0.00825142689119, 0.0, 1.04457382236;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.25738796225, 0.00756271754847, 74.8971101634, 1.58403340092, 0.00756271754847, 1.25738796225, 1.58403340092, 74.8971101634;
  Eigen::Matrix<double, 2, 4> K;
  K << 154.070413936, 12.2263299292, 2.23580489275, 0.77037863276, 2.23580489275, 0.77037863276, 154.070413936, 12.2263299292;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00511059808244, 0.0, 2.03320493463e-05, 0.0, 1.04457382236, 0.0, -0.00825142689119, 0.0, 2.03320493463e-05, 1.0, -0.00511059808244, 0.0, -0.00825142689119, 0.0, 1.04457382236;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainHighHighPlantCoefficients());
}

StateFeedbackPlant<4, 2, 2> MakeDrivetrainPlant() {
  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>> plants(4);
  plants[0] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainLowLowPlantCoefficients()));
  plants[1] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainLowHighPlantCoefficients()));
  plants[2] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainHighLowPlantCoefficients()));
  plants[3] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainHighHighPlantCoefficients()));
  return StateFeedbackPlant<4, 2, 2>(&plants);
}

StateFeedbackLoop<4, 2, 2> MakeDrivetrainLoop() {
  ::std::vector< ::std::unique_ptr<StateFeedbackController<4, 2, 2>>> controllers(4);
  controllers[0] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeDrivetrainLowLowController()));
  controllers[1] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeDrivetrainLowHighController()));
  controllers[2] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeDrivetrainHighLowController()));
  controllers[3] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeDrivetrainHighHighController()));
  return StateFeedbackLoop<4, 2, 2>(&controllers);
}

}  // namespace control_loops
}  // namespace frc971
