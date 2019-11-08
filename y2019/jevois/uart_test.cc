#include "y2019/jevois/uart.h"

#include <stdint.h>

#include "gtest/gtest.h"

namespace frc971 {
namespace jevois {
namespace testing {

// Tests packing and then unpacking a message with arbitrary values.
TEST(UartToTeensyTest, Basic) {
  CameraFrame input_message;
  for (int i = 0; i < 3; ++i) {
    input_message.targets.push_back({});
    Target *const target = &input_message.targets.back();
    target->distance = i * 7 + 1;
    target->height = i * 7 + 2;
    target->heading = i * 7 + 3;
    target->skew = i * 7 + 5;
  }
  input_message.age = camera_duration(123);
  const UartToTeensyBuffer buffer = UartPackToTeensy(input_message);
  const auto output_message = UartUnpackToTeensy(buffer);
  ASSERT_TRUE(output_message);
  EXPECT_EQ(input_message, output_message.value());
}

// Tests packing and then unpacking a message with arbitrary values and no
// frames.
TEST(UartToTeensyTest, NoFrames) {
  CameraFrame input_message;
  input_message.age = camera_duration(123);
  const UartToTeensyBuffer buffer = UartPackToTeensy(input_message);
  const auto output_message = UartUnpackToTeensy(buffer);
  ASSERT_TRUE(output_message);
  EXPECT_EQ(input_message, output_message.value());
}

// Tests packing and then unpacking a message with just one frame.
TEST(UartToTeensyTest, OneFrame) {
  CameraFrame input_message;
  {
    input_message.targets.push_back({});
    Target *const target = &input_message.targets.back();
    target->distance = 1;
    target->height = 2;
    target->heading = 3;
    target->skew = 5;
  }
  input_message.age = camera_duration(123);
  const UartToTeensyBuffer buffer = UartPackToTeensy(input_message);
  const auto output_message = UartUnpackToTeensy(buffer);
  ASSERT_TRUE(output_message);
  EXPECT_EQ(input_message, output_message.value());
}

// Tests packing and then unpacking a message with arbitrary values.
TEST(UartToCameraTest, Basic) {
  CameraCalibration input_message;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      input_message.calibration(i, j) = i * 5 + j * 971;
    }
  }
  input_message.teensy_now =
      aos::monotonic_clock::time_point(std::chrono::seconds(1678));
  input_message.realtime_now = aos::realtime_clock::min_time;
  input_message.camera_command = CameraCommand::kCameraPassthrough;
  const UartToCameraBuffer buffer = UartPackToCamera(input_message);
  const auto output_message = UartUnpackToCamera(buffer);
  ASSERT_TRUE(output_message);
  EXPECT_EQ(input_message, output_message.value());
}

// Tests that corrupting the data in various ways is handled properly.
TEST(UartToTeensyTest, CorruptData) {
  CameraFrame input_message{};
  {
    UartToTeensyBuffer buffer = UartPackToTeensy(input_message);
    buffer[0]++;
    EXPECT_FALSE(UartUnpackToTeensy(buffer));
  }
  {
    UartToTeensyBuffer buffer = UartPackToTeensy(input_message);
    buffer[buffer.size() - 1]++;
    EXPECT_FALSE(UartUnpackToTeensy(buffer));
  }
  {
    UartToTeensyBuffer buffer = UartPackToTeensy(input_message);
    buffer.set_size(buffer.size() - 1);
    EXPECT_FALSE(UartUnpackToTeensy(buffer));
  }
  {
    UartToTeensyBuffer buffer = UartPackToTeensy(input_message);
    buffer[0] = -1;
    EXPECT_FALSE(UartUnpackToTeensy(buffer));
  }
}

// Tests that corrupting the data in various ways is handled properly.
TEST(UartToCameraTest, CorruptData) {
  CameraCalibration input_message{};
  {
    UartToCameraBuffer buffer = UartPackToCamera(input_message);
    buffer[0]++;
    EXPECT_FALSE(UartUnpackToCamera(buffer));
  }
  {
    UartToCameraBuffer buffer = UartPackToCamera(input_message);
    buffer[buffer.size() - 1]++;
    EXPECT_FALSE(UartUnpackToCamera(buffer));
  }
  {
    UartToCameraBuffer buffer = UartPackToCamera(input_message);
    buffer.set_size(buffer.size() - 1);
    EXPECT_FALSE(UartUnpackToCamera(buffer));
  }
  {
    UartToCameraBuffer buffer = UartPackToCamera(input_message);
    buffer[0] = -1;
    EXPECT_FALSE(UartUnpackToCamera(buffer));
  }
}

}  // namespace testing
}  // namespace jevois
}  // namespace frc971
