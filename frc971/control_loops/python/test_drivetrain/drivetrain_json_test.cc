#include "gtest/gtest.h"

#include "aos/util/file.h"
#include "frc971/control_loops/drivetrain/drivetrain_config_static.h"
#include "frc971/control_loops/hybrid_state_feedback_loop_converters.h"
#include "frc971/control_loops/python/test_drivetrain/drivetrain_dog_motor_plant.h"
#include "frc971/control_loops/python/test_drivetrain/hybrid_velocity_drivetrain.h"
#include "frc971/control_loops/python/test_drivetrain/kalman_drivetrain_motor_plant.h"
#include "frc971/control_loops/python/test_drivetrain/polydrivetrain_dog_motor_plant.h"

namespace frc971::control_loops::drivetrain::testing {

class DrivetrainJsonTest : public ::testing::Test {
 protected:
  DrivetrainJsonTest() {}

  aos::FlatbufferDetachedBuffer<fbs::DrivetrainLoopConfig> ReadCoefficients(
      std::string_view file) {
    const std::string json = aos::util::ReadFileToStringOrDie(file);
    return aos::JsonToFlatbuffer<fbs::DrivetrainLoopConfig>(json);
  }
  aos::FlatbufferDetachedBuffer<fbs::DrivetrainLoopConfig>
  ReadHybridCoefficients(std::string_view file) {
    const std::string json = aos::util::ReadFileToStringOrDie(file);
    return aos::JsonToFlatbuffer<fbs::DrivetrainLoopConfig>(json);
  }
};

TEST_F(DrivetrainJsonTest, DrivetrainLoop) {
  StateFeedbackLoop<4, 2, 2> made_loop =
      python::test_drivetrain::MakeDrivetrainLoop();
  auto coeffs = ReadCoefficients(
      "frc971/control_loops/python/test_drivetrain/"
      "drivetrain_dog_motor_plant.json");

  StateFeedbackLoop<4, 2, 2> json_loop = MakeStateFeedbackLoop<4, 2, 2>(
      *CHECK_NOTNULL(coeffs.message().drivetrain_loop()));
  for (size_t index = 0; index < 4; ++index) {
    ASSERT_TRUE(coeffs.span().size() > 0u);
    made_loop.set_index(index);
    json_loop.set_index(index);
#define COMPARE(matrix)                              \
  EXPECT_EQ(json_loop.plant().coefficients().matrix, \
            made_loop.plant().coefficients().matrix);
    COMPARE(A);
    COMPARE(B);
    COMPARE(C);
    COMPARE(D);
    COMPARE(U_min);
    COMPARE(U_max);
    COMPARE(U_limit_coefficient);
    COMPARE(U_limit_constant);
    COMPARE(dt);
    COMPARE(delayed_u);
#undef COMPARE
#define COMPARE(matrix)                                   \
  EXPECT_EQ(json_loop.controller().coefficients().matrix, \
            made_loop.controller().coefficients().matrix);
    COMPARE(K);
    COMPARE(Kff);
#undef COMPARE
#define COMPARE(matrix)                                 \
  EXPECT_EQ(json_loop.observer().coefficients().matrix, \
            made_loop.observer().coefficients().matrix);
    COMPARE(KalmanGain);
    COMPARE(Q);
    COMPARE(R);
    COMPARE(delayed_u);
#undef COMPARE
  }
}

typedef StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                          HybridKalman<2, 2, 2>>
    HybridLoop;
TEST_F(DrivetrainJsonTest, HybridLoop) {
  HybridLoop made_loop =
      python::test_drivetrain::MakeHybridVelocityDrivetrainLoop();
  auto coeffs = ReadHybridCoefficients(
      "frc971/control_loops/python/test_drivetrain/"
      "hybrid_velocity_drivetrain.json");

  HybridLoop json_loop = MakeHybridStateFeedbackLoop<2, 2, 2>(
      *CHECK_NOTNULL(coeffs.message().hybrid_velocity_drivetrain_loop()));
  for (size_t index = 0; index < 4; ++index) {
    ASSERT_TRUE(coeffs.span().size() > 0u);
    made_loop.set_index(index);
    json_loop.set_index(index);
#define COMPARE(matrix)                              \
  EXPECT_EQ(json_loop.plant().coefficients().matrix, \
            made_loop.plant().coefficients().matrix);
    COMPARE(A_continuous);
    COMPARE(B_continuous);
    COMPARE(C);
    COMPARE(D);
    COMPARE(U_min);
    COMPARE(U_max);
    COMPARE(U_limit_coefficient);
    COMPARE(U_limit_constant);
    COMPARE(delayed_u);
#undef COMPARE
#define COMPARE(matrix)                                   \
  EXPECT_EQ(json_loop.controller().coefficients().matrix, \
            made_loop.controller().coefficients().matrix);
    COMPARE(K);
    COMPARE(Kff);
#undef COMPARE
#define COMPARE(matrix)                                 \
  EXPECT_EQ(json_loop.observer().coefficients().matrix, \
            made_loop.observer().coefficients().matrix);
    COMPARE(Q_continuous);
    COMPARE(R_continuous);
    COMPARE(P_steady_state);
    COMPARE(delayed_u);
#undef COMPARE
  }
}
}  // namespace frc971::control_loops::drivetrain::testing
