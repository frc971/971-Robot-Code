#include "y2019/jevois/spi.h"

#include <stdint.h>

#include "gtest/gtest.h"

namespace frc971 {
namespace jevois {
namespace testing {

// Tests packing and then unpacking an empty message.
TEST(SpiToRoborioPackTest, Empty) {
  TeensyToRoborio input_message;
  const SpiTransfer transfer = SpiPackToRoborio(input_message);
  const auto output_message = SpiUnpackToRoborio(transfer);
  ASSERT_TRUE(output_message);
  EXPECT_EQ(input_message, output_message.value());
}

// Tests that unpacking after the message has been modified results in a
// checksum failure.
TEST(SpiToRoborioPackTest, CorruptChecksum) {
  TeensyToRoborio input_message;
  {
    SpiTransfer transfer = SpiPackToRoborio(input_message);
    transfer[0]++;
    ASSERT_FALSE(SpiUnpackToRoborio(transfer));
  }
  {
    SpiTransfer transfer = SpiPackToRoborio(input_message);
    transfer[0] ^= 0xFF;
    ASSERT_FALSE(SpiUnpackToRoborio(transfer));
  }
  {
    SpiTransfer transfer = SpiPackToRoborio(input_message);
    transfer[transfer.size() - 1]++;
    ASSERT_FALSE(SpiUnpackToRoborio(transfer));
  }
  input_message.frames.push_back({});
  {
    SpiTransfer transfer = SpiPackToRoborio(input_message);
    transfer[0]++;
    ASSERT_FALSE(SpiUnpackToRoborio(transfer));
  }
  {
    SpiTransfer transfer = SpiPackToRoborio(input_message);
    transfer[3]++;
    ASSERT_FALSE(SpiUnpackToRoborio(transfer));
  }
  input_message.frames.back().targets.push_back({});
  {
    SpiTransfer transfer = SpiPackToRoborio(input_message);
    transfer[3]++;
    ASSERT_FALSE(SpiUnpackToRoborio(transfer));
  }
}

// Tests packing and then unpacking a full message.
TEST(SpiToRoborioPackTest, Full) {
  TeensyToRoborio input_message;
  input_message.frames.push_back({});
  input_message.frames.back().age = camera_duration(9);
  input_message.frames.back().camera_index = 2;
  input_message.frames.push_back({});
  input_message.frames.back().age = camera_duration(7);
  input_message.frames.back().camera_index = 5;
  input_message.frames.push_back({});
  input_message.frames.back().age = camera_duration(1);
  input_message.frames.back().camera_index = 4;

  const SpiTransfer transfer = SpiPackToRoborio(input_message);
  const auto output_message = SpiUnpackToRoborio(transfer);
  ASSERT_TRUE(output_message);
  EXPECT_EQ(input_message, output_message.value());
}

// Tests that packing and unpacking a target results in values close to before.
TEST(SpiToRoborioPackTest, Target) {
  TeensyToRoborio input_message;
  input_message.frames.push_back({});
  input_message.frames.back().targets.push_back({});
  input_message.frames.back().targets.back().distance = 9;
  input_message.frames.back().targets.back().height = 1;
  input_message.frames.back().targets.back().heading = 0.5;
  input_message.frames.back().targets.back().skew = -0.5;
  input_message.frames.back().camera_index = 0;
  input_message.frames.push_back({});
  input_message.frames.back().targets.push_back({});
  input_message.frames.back().targets.push_back({});
  input_message.frames.back().camera_index = 2;
  input_message.frames.push_back({});
  input_message.frames.back().targets.push_back({});
  input_message.frames.back().targets.push_back({});
  input_message.frames.back().targets.push_back({});
  input_message.frames.back().camera_index = 3;

  const SpiTransfer transfer = SpiPackToRoborio(input_message);
  const auto output_message = SpiUnpackToRoborio(transfer);
  ASSERT_TRUE(output_message);
  ASSERT_EQ(3u, output_message->frames.size());
  ASSERT_EQ(1u, output_message->frames[0].targets.size());
  ASSERT_EQ(2u, output_message->frames[1].targets.size());
  ASSERT_EQ(3u, output_message->frames[2].targets.size());
  EXPECT_NEAR(input_message.frames.back().targets.back().distance,
              output_message->frames.back().targets.back().distance, 0.1);
  EXPECT_NEAR(input_message.frames.back().targets.back().height,
              output_message->frames.back().targets.back().height, 0.1);
  EXPECT_NEAR(input_message.frames.back().targets.back().heading,
              output_message->frames.back().targets.back().heading, 0.1);
  EXPECT_NEAR(input_message.frames.back().targets.back().skew,
              output_message->frames.back().targets.back().skew, 0.1);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(input_message.frames[i].camera_index,
              output_message->frames[i].camera_index);
  }
}

// Tests that packing and unpacking two targets results in the same number on
// the other side.
TEST(SpiToRoborioPackTest, TwoTargets) {
  TeensyToRoborio input_message;
  input_message.frames.push_back({});
  input_message.frames.back().targets.push_back({});
  input_message.frames.back().targets.back().distance = 9;
  input_message.frames.back().targets.back().height = 1;
  input_message.frames.back().targets.back().heading = 0.5;
  input_message.frames.back().targets.back().skew = -0.5;
  input_message.frames.back().targets.push_back({});
  input_message.frames.back().targets.back().distance = 1;
  input_message.frames.back().targets.back().height = 0.9;
  input_message.frames.back().targets.back().heading = 0.4;
  input_message.frames.back().targets.back().skew = -0.4;
  input_message.frames.back().age = camera_duration(9);
  input_message.frames.back().camera_index = 2;

  const SpiTransfer transfer = SpiPackToRoborio(input_message);
  const auto output_message = SpiUnpackToRoborio(transfer);
  ASSERT_TRUE(output_message);
  ASSERT_EQ(1u, output_message->frames.size());
  ASSERT_EQ(2u, output_message->frames[0].targets.size());
  for (int i = 0; i < 2; ++i) {
    EXPECT_NEAR(input_message.frames.back().targets[i].distance,
                output_message->frames.back().targets[i].distance, 0.1);
    EXPECT_NEAR(input_message.frames.back().targets[i].height,
                output_message->frames.back().targets[i].height, 0.1);
    EXPECT_NEAR(input_message.frames.back().targets[i].heading,
                output_message->frames.back().targets[i].heading, 0.1);
    EXPECT_NEAR(input_message.frames.back().targets[i].skew,
                output_message->frames.back().targets[i].skew, 0.1);
    EXPECT_EQ(input_message.frames.back().age,
              output_message->frames.back().age);
    EXPECT_EQ(input_message.frames.back().camera_index,
              output_message->frames.back().camera_index);
  }
}

// Tests that packing and unpacking three targets results in the same number on
// the other side.
TEST(SpiToRoborioPackTest, ThreeTargets) {
  TeensyToRoborio input_message;
  input_message.frames.push_back({});
  input_message.frames.back().targets.push_back({});
  input_message.frames.back().targets.back().distance = 9;
  input_message.frames.back().targets.back().height = 1;
  input_message.frames.back().targets.back().heading = 0.5;
  input_message.frames.back().targets.back().skew = -0.5;
  input_message.frames.back().targets.push_back({});
  input_message.frames.back().targets.back().distance = 1;
  input_message.frames.back().targets.back().height = 0.9;
  input_message.frames.back().targets.back().heading = 0.4;
  input_message.frames.back().targets.back().skew = -0.4;
  input_message.frames.back().targets.push_back({});
  input_message.frames.back().targets.back().distance = 2;
  input_message.frames.back().targets.back().height = 0.7;
  input_message.frames.back().targets.back().heading = 0.3;
  input_message.frames.back().targets.back().skew = -0.3;
  input_message.frames.back().age = camera_duration(1);
  input_message.frames.back().camera_index = 1;

  const SpiTransfer transfer = SpiPackToRoborio(input_message);
  const auto output_message = SpiUnpackToRoborio(transfer);
  ASSERT_TRUE(output_message);
  ASSERT_EQ(1u, output_message->frames.size());
  ASSERT_EQ(3u, output_message->frames[0].targets.size());
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(input_message.frames.back().targets[i].distance,
                output_message->frames.back().targets[i].distance, 0.1);
    EXPECT_NEAR(input_message.frames.back().targets[i].height,
                output_message->frames.back().targets[i].height, 0.1);
    EXPECT_NEAR(input_message.frames.back().targets[i].heading,
                output_message->frames.back().targets[i].heading, 0.1);
    EXPECT_NEAR(input_message.frames.back().targets[i].skew,
                output_message->frames.back().targets[i].skew, 0.1);
    EXPECT_EQ(input_message.frames.back().age,
              output_message->frames.back().age);
    EXPECT_EQ(input_message.frames.back().camera_index,
              output_message->frames.back().camera_index);
  }
}

// Tests packing and then unpacking an empty message.
TEST(SpiToTeensyPackTest, Empty) {
  RoborioToTeensy input_message{};
  const SpiTransfer transfer = SpiPackToTeensy(input_message);
  const auto output_message = SpiUnpackToTeensy(transfer);
  ASSERT_TRUE(output_message);
  EXPECT_EQ(input_message, output_message.value());
}

// Tests packing and then unpacking a message with all the fields set.
TEST(SpiToTeensyPackTest, Full) {
  RoborioToTeensy input_message;
  input_message.beacon_brightness[0] = 9;
  input_message.beacon_brightness[1] = 7;
  input_message.beacon_brightness[2] = 1;
  input_message.light_rings[0] = 1;
  input_message.light_rings[1] = 0;
  input_message.light_rings[2] = 0;
  input_message.light_rings[3] = 1;
  input_message.light_rings[4] = 0;
  input_message.realtime_now =
      aos::realtime_clock::epoch() + std::chrono::seconds(971254);
  input_message.camera_command = CameraCommand::kUsb;

  const SpiTransfer transfer = SpiPackToTeensy(input_message);
  const auto output_message = SpiUnpackToTeensy(transfer);
  ASSERT_TRUE(output_message);
  EXPECT_EQ(input_message, output_message.value());
}

}  // namespace testing
}  // namespace jevois
}  // namespace frc971
