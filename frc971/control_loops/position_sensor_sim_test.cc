#include <unistd.h>

#include <memory>

#include <random>

#include "gtest/gtest.h"
#include "frc971/control_loops/control_loops.q.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "aos/common/die.h"

namespace frc971 {
namespace control_loops {

class PositionSensorSimTest : public ::testing::Test {
 protected:
  PositionSensorSimTest() {}
};

TEST_F(PositionSensorSimTest, NoIndices) {
  // We'll simulate a potentiometer with no noise so that we can accurately
  // verify where the mechanism currently is. Overall though, the purpose of
  // this test is to verify that no false index pulses are generated while the
  // mechanism stays between two index pulses.
  const double index_diff = 0.5;
  PotAndIndexPosition position;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(3.6 * index_diff, 0);

  // Make sure that we don't accidentally hit an index pulse.
  for (int i = 0; i < 30; i++) {
    sim.MoveTo(3.6 * index_diff);
    sim.GetSensorValues(&position);
    ASSERT_DOUBLE_EQ(3.6 * index_diff, position.pot);
    ASSERT_EQ(0u, position.index_pulses);
  }

  for (int i = 0; i < 30; i++) {
    sim.MoveTo(3.0 * index_diff);
    sim.GetSensorValues(&position);
    ASSERT_DOUBLE_EQ(3.0 * index_diff, position.pot);
    ASSERT_EQ(0u, position.index_pulses);
  }

  for (int i = 0; i < 30; i++) {
    sim.MoveTo(3.99 * index_diff);
    sim.GetSensorValues(&position);
    ASSERT_DOUBLE_EQ(3.99 * index_diff, position.pot);
    ASSERT_EQ(0u, position.index_pulses);
  }

  for (int i = 0; i < 30; i++) {
    sim.MoveTo(3.0 * index_diff);
    sim.GetSensorValues(&position);
    ASSERT_DOUBLE_EQ(3.0 * index_diff, position.pot);
    ASSERT_EQ(0u, position.index_pulses);
  }
}

TEST_F(PositionSensorSimTest, CountIndices) {
  // The purpose of this test is to verify that the simulator latches the
  // correct index pulse when transitioning from one segment to another. We
  // again simulate zero noise on the potentiometer to accurately verify the
  // mechanism's position during the index pulses.
  const double index_diff = 0.8;
  PotAndIndexPosition position;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(4.6 * index_diff, 0);

  // Make sure that we get an index pulse on every transition.
  sim.GetSensorValues(&position);
  ASSERT_EQ(0u, position.index_pulses);

  sim.MoveTo(3.6 * index_diff);
  sim.GetSensorValues(&position);
  ASSERT_DOUBLE_EQ(4.0 * index_diff, position.latched_pot);
  ASSERT_EQ(1u, position.index_pulses);

  sim.MoveTo(4.5 * index_diff);
  sim.GetSensorValues(&position);
  ASSERT_DOUBLE_EQ(4.0 * index_diff, position.latched_pot);
  ASSERT_EQ(2u, position.index_pulses);

  sim.MoveTo(5.9 * index_diff);
  sim.GetSensorValues(&position);
  ASSERT_DOUBLE_EQ(5.0 * index_diff, position.latched_pot);
  ASSERT_EQ(3u, position.index_pulses);

  sim.MoveTo(6.1 * index_diff);
  sim.GetSensorValues(&position);
  ASSERT_DOUBLE_EQ(6.0 * index_diff, position.latched_pot);
  ASSERT_EQ(4u, position.index_pulses);

  sim.MoveTo(8.7 * index_diff);
  sim.GetSensorValues(&position);
  ASSERT_DOUBLE_EQ(8.0 * index_diff, position.latched_pot);
  ASSERT_EQ(5u, position.index_pulses);

  sim.MoveTo(7.3 * index_diff);
  sim.GetSensorValues(&position);
  ASSERT_DOUBLE_EQ(8.0 * index_diff, position.latched_pot);
  ASSERT_EQ(6u, position.index_pulses);
}

// Tests that the simulator handles non-zero specified index pulse locations
// correctly.
TEST_F(PositionSensorSimTest, NonZeroIndexLocation) {
  const double index_diff = 0.5;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(index_diff * 0.25, 0.0, index_diff * 0.5);
  PotAndIndexPosition position;

  sim.MoveTo(0.75 * index_diff);
  sim.GetSensorValues(&position);
  EXPECT_EQ(1u, position.index_pulses);
  EXPECT_DOUBLE_EQ(index_diff * 0.5, position.latched_pot);
  EXPECT_DOUBLE_EQ(index_diff * 0.25, position.latched_encoder);

  sim.MoveTo(index_diff);
  sim.GetSensorValues(&position);
  EXPECT_EQ(1u, position.index_pulses);
  EXPECT_DOUBLE_EQ(index_diff * 0.5, position.latched_pot);
  EXPECT_DOUBLE_EQ(index_diff * 0.25, position.latched_encoder);

  sim.MoveTo(1.75 * index_diff);
  sim.GetSensorValues(&position);
  EXPECT_EQ(2u, position.index_pulses);
  EXPECT_DOUBLE_EQ(index_diff * 1.5, position.latched_pot);
  EXPECT_DOUBLE_EQ(index_diff * 1.25, position.latched_encoder);

  // Try it with our known index pulse not being our first one.
  sim.Initialize(index_diff * 0.25, 0.0, index_diff * 1.5);

  sim.MoveTo(0.75 * index_diff);
  sim.GetSensorValues(&position);
  EXPECT_EQ(1u, position.index_pulses);
  EXPECT_DOUBLE_EQ(index_diff * 0.5, position.latched_pot);
  EXPECT_DOUBLE_EQ(index_diff * 0.25, position.latched_encoder);

  sim.MoveTo(index_diff);
  sim.GetSensorValues(&position);
  EXPECT_EQ(1u, position.index_pulses);
  EXPECT_DOUBLE_EQ(index_diff * 0.5, position.latched_pot);
  EXPECT_DOUBLE_EQ(index_diff * 0.25, position.latched_encoder);

  sim.MoveTo(1.75 * index_diff);
  sim.GetSensorValues(&position);
  EXPECT_EQ(2u, position.index_pulses);
  EXPECT_DOUBLE_EQ(index_diff * 1.5, position.latched_pot);
  EXPECT_DOUBLE_EQ(index_diff * 1.25, position.latched_encoder);
}

}  // namespace control_loops
}  // namespace frc971
