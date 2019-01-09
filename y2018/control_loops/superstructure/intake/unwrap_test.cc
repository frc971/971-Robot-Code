#include "gtest/gtest.h"
#include "y2018/control_loops/superstructure/intake/sensor_unwrap.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace intake {
namespace testing {

TEST(SensorTest, UnwrapOnce) {
  // Test the sensor moving over the maximum range value and wrapping once
  // then move sensor in oppsite direction to unwrap and test result.

  // Initialize with the offset and range
  UnwrapSensor sensor(-1.5, 6);        // min = -1.5 & max = 4.5 & move > 3
  EXPECT_EQ(sensor.Unwrap(1.5), 1.5);  //  move n/a
  EXPECT_EQ(sensor.sensor_wrapped(), 0);
  EXPECT_EQ(sensor.Unwrap(4.0), 4.0);   // move 2.5
  EXPECT_EQ(sensor.Unwrap(-1.0), 5.0);  // move -5.0 -> wrap+
  EXPECT_EQ(sensor.sensor_wrapped(), 1);
  EXPECT_EQ(sensor.Unwrap(-1.5), 4.5);  // move -0.5
  EXPECT_EQ(sensor.Unwrap(4.0), 4.0);   // move 5.5 -> wrap-
  EXPECT_EQ(sensor.sensor_wrapped(), 0);
  sensor.Reset();
}

TEST(SensorTest, UnwrapTwice) {
  // Test the sensor wrapping twice over the lower value of the range.

  // Initialize with the offset and range
  UnwrapSensor sensor(-1.5, 6);
  EXPECT_EQ(sensor.Unwrap(1.0), 1.0);
  EXPECT_EQ(sensor.Unwrap(-1.0), -1.0);
  EXPECT_EQ(sensor.Unwrap(4.0), -2.0);
  EXPECT_EQ(sensor.sensor_wrapped(), -1);
  EXPECT_EQ(sensor.Unwrap(2.0), -4.0);
  EXPECT_EQ(sensor.Unwrap(-1.0), -7.0);
  EXPECT_EQ(sensor.Unwrap(4.0), -8.0);
  EXPECT_EQ(sensor.sensor_wrapped(), -2);
}

TEST(SensorTest, UnwrapOutRange) {
  // Test if values out side range are handled proporly.
  // Not wrapped scenario only.

  UnwrapSensor sensor(-1.5, 6);
  EXPECT_EQ(sensor.Unwrap(-3.0), -3.0);  // Passed by the init stage
  EXPECT_EQ(sensor.Unwrap(-3.0), -3.0);  // Caught by the exeption handler
  EXPECT_EQ(sensor.Unwrap(6.5), 6.5);
}

TEST(SensorTest, UnwrapInit) {
  // Test the case where the start value and offset will be far enough apart to
  // trigger a wrap. By ignoring the fisrt value for evaluation and set that for
  // the next evaluation, this should not trigger the wrap.

  UnwrapSensor sensor(-0.6, 1.0);      // min = -0.6 & max = 0.4 & move > 0.5
  EXPECT_EQ(sensor.Unwrap(0.0), 0.0);  // move = n/a
  EXPECT_EQ(sensor.sensor_wrapped(), 0);
  EXPECT_EQ(sensor.Unwrap(0.0), 0.0);
  sensor.Reset();
  EXPECT_EQ(sensor.Unwrap(0.4), 0.4);   // move = n/a
  EXPECT_EQ(sensor.Unwrap(-0.4), 0.6);  // move = -0.8, wrap 1
  EXPECT_EQ(sensor.Unwrap(0.2), 0.2);   // move = 1.0, wrap -1
}

}  // namespace testing
}  // namespace intake
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
