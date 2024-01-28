#include "gtest/gtest.h"

#include "aos/testing/path.h"
#include "frc971/control_loops/drivetrain/drivetrain_config_static.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"

namespace frc971::control_loops::drivetrain::testing {

// Validate that we can correctly convert a flatbuffer drivetrain configuration
// into a DrivetrainConfig object.
TEST(DrivetrainConfigTest, FlatbufferConversionTest) {
  auto json_config = DrivetrainConfig<double>::FromFlatbuffer(
      aos::JsonFileToFlatbuffer<fbs::DrivetrainConfig>(
          aos::testing::ArtifactPath(
              "frc971/control_loops/drivetrain/drivetrain_test_config.json"))
          .message());
  auto cpp_config = GetTestDrivetrainConfig();
#define COMPARE(name) EXPECT_EQ(json_config.name, cpp_config.name);
#define COMPARE_FLOAT(name) \
  EXPECT_NEAR(json_config.name, cpp_config.name, 1e-6);
  COMPARE(shifter_type);
  COMPARE(loop_type);
  COMPARE(gyro_type);
  COMPARE(imu_type);
  COMPARE(dt);
  COMPARE_FLOAT(robot_radius);
  COMPARE_FLOAT(wheel_radius);
  COMPARE_FLOAT(v);
  COMPARE_FLOAT(high_gear_ratio);
  COMPARE_FLOAT(low_gear_ratio);
  COMPARE_FLOAT(J);
  COMPARE_FLOAT(mass);
  COMPARE(left_drive);
  COMPARE(right_drive);
  COMPARE(default_high_gear);
  COMPARE_FLOAT(down_offset);
  COMPARE_FLOAT(wheel_non_linearity);
  COMPARE_FLOAT(quickturn_wheel_multiplier);
  COMPARE_FLOAT(wheel_multiplier);
  COMPARE(pistol_grip_shift_enables_line_follow);
  COMPARE(imu_transform);
  COMPARE(is_simulated);
  COMPARE(down_estimator_config);
  COMPARE(line_follow_config.Q);
  COMPARE(line_follow_config.R);
  COMPARE(line_follow_config.max_controllable_offset);
  COMPARE(top_button_use);
  COMPARE(second_button_use);
  COMPARE(bottom_button_use);
#undef COMPARE
#undef COMPARE_FLOAT
}
}  // namespace frc971::control_loops::drivetrain::testing
