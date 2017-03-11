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
  IndexPosition index_position;
  PotAndIndexPosition pot_and_index_position;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(3.6 * index_diff, 0);

  // Make sure that we don't accidentally hit an index pulse.
  for (int i = 0; i < 30; i++) {
    sim.MoveTo(3.6 * index_diff);
    sim.GetSensorValues(&index_position);
    sim.GetSensorValues(&pot_and_index_position);
    ASSERT_DOUBLE_EQ(3.6 * index_diff, pot_and_index_position.pot);
    ASSERT_EQ(0u, pot_and_index_position.index_pulses);
    ASSERT_EQ(0u, index_position.index_pulses);
  }

  for (int i = 0; i < 30; i++) {
    sim.MoveTo(3.0 * index_diff);
    sim.GetSensorValues(&index_position);
    sim.GetSensorValues(&pot_and_index_position);
    ASSERT_DOUBLE_EQ(3.0 * index_diff, pot_and_index_position.pot);
    ASSERT_EQ(0u, pot_and_index_position.index_pulses);
    ASSERT_EQ(0u, index_position.index_pulses);
  }

  for (int i = 0; i < 30; i++) {
    sim.MoveTo(3.99 * index_diff);
    sim.GetSensorValues(&index_position);
    sim.GetSensorValues(&pot_and_index_position);
    ASSERT_DOUBLE_EQ(3.99 * index_diff, pot_and_index_position.pot);
    ASSERT_EQ(0u, pot_and_index_position.index_pulses);
    ASSERT_EQ(0u, index_position.index_pulses);
  }

  for (int i = 0; i < 30; i++) {
    sim.MoveTo(3.0 * index_diff);
    sim.GetSensorValues(&index_position);
    sim.GetSensorValues(&pot_and_index_position);
    ASSERT_DOUBLE_EQ(3.0 * index_diff, pot_and_index_position.pot);
    ASSERT_EQ(0u, pot_and_index_position.index_pulses);
    ASSERT_EQ(0u, index_position.index_pulses);
  }
}

TEST_F(PositionSensorSimTest, CountIndices) {
  // The purpose of this test is to verify that the simulator latches the
  // correct index pulse when transitioning from one segment to another. We
  // again simulate zero noise on the potentiometer to accurately verify the
  // mechanism's position during the index pulses.
  const double index_diff = 0.8;
  IndexPosition index_position;
  PotAndIndexPosition pot_and_index_position;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(4.6 * index_diff, 0);

  // Make sure that we get an index pulse on every transition.
  sim.GetSensorValues(&index_position);
  sim.GetSensorValues(&pot_and_index_position);
  ASSERT_EQ(0u, index_position.index_pulses);
  ASSERT_EQ(0u, pot_and_index_position.index_pulses);

  sim.MoveTo(3.6 * index_diff);
  sim.GetSensorValues(&index_position);
  sim.GetSensorValues(&pot_and_index_position);
  ASSERT_DOUBLE_EQ(4.0 * index_diff, pot_and_index_position.latched_pot);
  ASSERT_EQ(1u, index_position.index_pulses);
  ASSERT_EQ(1u, pot_and_index_position.index_pulses);

  sim.MoveTo(4.5 * index_diff);
  sim.GetSensorValues(&index_position);
  sim.GetSensorValues(&pot_and_index_position);
  ASSERT_DOUBLE_EQ(4.0 * index_diff, pot_and_index_position.latched_pot);
  ASSERT_EQ(2u, index_position.index_pulses);
  ASSERT_EQ(2u, pot_and_index_position.index_pulses);

  sim.MoveTo(5.9 * index_diff);
  sim.GetSensorValues(&index_position);
  sim.GetSensorValues(&pot_and_index_position);
  ASSERT_DOUBLE_EQ(5.0 * index_diff, pot_and_index_position.latched_pot);
  ASSERT_EQ(3u, index_position.index_pulses);
  ASSERT_EQ(3u, pot_and_index_position.index_pulses);

  sim.MoveTo(6.1 * index_diff);
  sim.GetSensorValues(&index_position);
  sim.GetSensorValues(&pot_and_index_position);
  ASSERT_DOUBLE_EQ(6.0 * index_diff, pot_and_index_position.latched_pot);
  ASSERT_EQ(4u, index_position.index_pulses);
  ASSERT_EQ(4u, pot_and_index_position.index_pulses);

  sim.MoveTo(8.7 * index_diff);
  sim.GetSensorValues(&index_position);
  sim.GetSensorValues(&pot_and_index_position);
  ASSERT_DOUBLE_EQ(8.0 * index_diff, pot_and_index_position.latched_pot);
  ASSERT_EQ(5u, index_position.index_pulses);
  ASSERT_EQ(5u, pot_and_index_position.index_pulses);

  sim.MoveTo(7.3 * index_diff);
  sim.GetSensorValues(&index_position);
  sim.GetSensorValues(&pot_and_index_position);
  ASSERT_DOUBLE_EQ(8.0 * index_diff, pot_and_index_position.latched_pot);
  ASSERT_EQ(6u, index_position.index_pulses);
  ASSERT_EQ(6u, pot_and_index_position.index_pulses);
}

// Tests that the simulator handles non-zero specified index pulse locations
// correctly.
TEST_F(PositionSensorSimTest, NonZeroIndexLocation) {
  const double index_diff = 0.5;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(index_diff * 0.25, 0.0, index_diff * 0.5);
  IndexPosition index_position;
  PotAndIndexPosition pot_and_index_position;

  sim.MoveTo(0.75 * index_diff);
  sim.GetSensorValues(&index_position);
  sim.GetSensorValues(&pot_and_index_position);
  EXPECT_EQ(1u, index_position.index_pulses);
  EXPECT_EQ(1u, pot_and_index_position.index_pulses);
  EXPECT_DOUBLE_EQ(index_diff * 0.5, pot_and_index_position.latched_pot);
  EXPECT_DOUBLE_EQ(index_diff * 0.25, index_position.latched_encoder);
  EXPECT_DOUBLE_EQ(index_diff * 0.25, pot_and_index_position.latched_encoder);

  sim.MoveTo(index_diff);
  sim.GetSensorValues(&index_position);
  sim.GetSensorValues(&pot_and_index_position);
  EXPECT_EQ(1u, index_position.index_pulses);
  EXPECT_EQ(1u, pot_and_index_position.index_pulses);
  EXPECT_DOUBLE_EQ(index_diff * 0.5, pot_and_index_position.latched_pot);
  EXPECT_DOUBLE_EQ(index_diff * 0.25, index_position.latched_encoder);
  EXPECT_DOUBLE_EQ(index_diff * 0.25, pot_and_index_position.latched_encoder);

  sim.MoveTo(1.75 * index_diff);
  sim.GetSensorValues(&index_position);
  sim.GetSensorValues(&pot_and_index_position);
  EXPECT_EQ(2u, index_position.index_pulses);
  EXPECT_EQ(2u, pot_and_index_position.index_pulses);
  EXPECT_DOUBLE_EQ(index_diff * 1.5, pot_and_index_position.latched_pot);
  EXPECT_DOUBLE_EQ(index_diff * 1.25, index_position.latched_encoder);
  EXPECT_DOUBLE_EQ(index_diff * 1.25, pot_and_index_position.latched_encoder);

  // Try it with our known index pulse not being our first one.
  sim.Initialize(index_diff * 0.25, 0.0, index_diff * 1.5);

  sim.MoveTo(0.75 * index_diff);
  sim.GetSensorValues(&index_position);
  sim.GetSensorValues(&pot_and_index_position);
  EXPECT_EQ(1u, index_position.index_pulses);
  EXPECT_EQ(1u, pot_and_index_position.index_pulses);
  EXPECT_DOUBLE_EQ(index_diff * 0.5, pot_and_index_position.latched_pot);
  EXPECT_DOUBLE_EQ(index_diff * 0.25, index_position.latched_encoder);
  EXPECT_DOUBLE_EQ(index_diff * 0.25, pot_and_index_position.latched_encoder);

  sim.MoveTo(index_diff);
  sim.GetSensorValues(&index_position);
  sim.GetSensorValues(&pot_and_index_position);
  EXPECT_EQ(1u, index_position.index_pulses);
  EXPECT_EQ(1u, pot_and_index_position.index_pulses);
  EXPECT_DOUBLE_EQ(index_diff * 0.5, pot_and_index_position.latched_pot);
  EXPECT_DOUBLE_EQ(index_diff * 0.25, index_position.latched_encoder);
  EXPECT_DOUBLE_EQ(index_diff * 0.25, pot_and_index_position.latched_encoder);

  sim.MoveTo(1.75 * index_diff);
  sim.GetSensorValues(&index_position);
  sim.GetSensorValues(&pot_and_index_position);
  EXPECT_EQ(2u, index_position.index_pulses);
  EXPECT_EQ(2u, pot_and_index_position.index_pulses);
  EXPECT_DOUBLE_EQ(index_diff * 1.5, pot_and_index_position.latched_pot);
  EXPECT_DOUBLE_EQ(index_diff * 1.25, index_position.latched_encoder);
  EXPECT_DOUBLE_EQ(index_diff * 1.25, pot_and_index_position.latched_encoder);
}

// Tests that the latched values update correctly.
TEST_F(PositionSensorSimTest, LatchedValues) {
  const double index_diff = 0.5;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(0, 0.25);
  PotAndIndexPosition position;

  sim.MoveTo(0.75 * index_diff);
  sim.GetSensorValues(&position);
  EXPECT_EQ(0u, position.index_pulses);

  sim.MoveTo(1.75 * index_diff);
  sim.GetSensorValues(&position);
  EXPECT_EQ(1u, position.index_pulses);
  EXPECT_NEAR(index_diff, position.latched_pot, 0.75);
  EXPECT_DOUBLE_EQ(index_diff, position.latched_encoder);
  const double first_latched_pot = position.latched_pot;

  sim.MoveTo(1.95 * index_diff);
  sim.GetSensorValues(&position);
  EXPECT_EQ(1u, position.index_pulses);
  EXPECT_NEAR(index_diff, position.latched_pot, 0.75);
  EXPECT_DOUBLE_EQ(first_latched_pot, position.latched_pot);
  EXPECT_DOUBLE_EQ(index_diff, position.latched_encoder);

  sim.MoveTo(2.05 * index_diff);
  sim.GetSensorValues(&position);
  EXPECT_EQ(2u, position.index_pulses);
  EXPECT_NEAR(index_diff * 2, position.latched_pot, 0.75);
  EXPECT_DOUBLE_EQ(index_diff * 2, position.latched_encoder);

  sim.MoveTo(1.95 * index_diff);
  sim.GetSensorValues(&position);
  EXPECT_EQ(3u, position.index_pulses);
  EXPECT_NEAR(index_diff * 2, position.latched_pot, 0.75);
  EXPECT_DOUBLE_EQ(index_diff * 2, position.latched_encoder);

  sim.MoveTo(0.95 * index_diff);
  sim.GetSensorValues(&position);
  EXPECT_EQ(4u, position.index_pulses);
  EXPECT_NEAR(index_diff, position.latched_pot, 0.75);
  EXPECT_GT(::std::abs(first_latched_pot - position.latched_pot), 0.005);
  EXPECT_DOUBLE_EQ(index_diff, position.latched_encoder);
}

// This test makes sure that our simulation for an absolute encoder + relative
// encoder + pot combo works OK. Let's pretend that we know that a reading of
// 0.07m on the absolute encoder corresponds to 0.2m on the robot. We also know
// that every 0.1m the absolute encoder resets. Then we can construct a table
// quickly from there:
//
// abs_encoder | robot
//     0.07m   |  0.20m
//     0.07m   |  0.30m
//     0.07m   |  0.40m
//     0.01m   |  0.34m
//     0.01m   |  0.24m
//     0.00m   |  0.23m
//     0.00m   |  0.13m
//
// Since the absolute encoder wraps around, we'll notice that the same reading
// can correspond to multiple positions on the robot.
//
// NOTE: We use EXPECT_NEAR rather than EXPECT_DOUBLE_EQ for the absolute
// encoder because the modulo operation inside the simulator introduces just
// enough imprecision to fail the EXPECT_DOUBLE_EQ macro.
TEST_F(PositionSensorSimTest, PotAndEncodersNoIndexPulse) {
  const double index_diff = 0.1;
  PositionSensorSimulator sim(index_diff);
  sim.Initialize(0.20, 0.05, 0.2, 0.07);
  PotAndAbsolutePosition position;

  sim.MoveTo(0.20);
  sim.GetSensorValues(&position);
  EXPECT_DOUBLE_EQ(0.00, position.encoder);
  EXPECT_NEAR(0.07, position.absolute_encoder, 0.00000001);

  sim.MoveTo(0.30);
  sim.GetSensorValues(&position);
  EXPECT_DOUBLE_EQ(0.10, position.encoder);
  EXPECT_NEAR(0.07, position.absolute_encoder, 0.00000001);

  sim.MoveTo(0.40);
  sim.GetSensorValues(&position);
  EXPECT_DOUBLE_EQ(0.20, position.encoder);
  EXPECT_NEAR(0.07, position.absolute_encoder, 0.00000001);

  sim.MoveTo(0.34);
  sim.GetSensorValues(&position);
  EXPECT_DOUBLE_EQ(0.14, position.encoder);
  EXPECT_NEAR(0.01, position.absolute_encoder, 0.00000001);

  sim.MoveTo(0.24);
  sim.GetSensorValues(&position);
  EXPECT_DOUBLE_EQ(0.04, position.encoder);
  EXPECT_NEAR(0.01, position.absolute_encoder, 0.00000001);

  sim.MoveTo(0.23);
  sim.GetSensorValues(&position);
  EXPECT_DOUBLE_EQ(0.03, position.encoder);
  EXPECT_NEAR(0.00, position.absolute_encoder, 0.00000001);

  sim.MoveTo(0.13);
  sim.GetSensorValues(&position);
  EXPECT_DOUBLE_EQ(-0.07, position.encoder);
  EXPECT_NEAR(0.00, position.absolute_encoder, 0.00000001);

  // TODO(philipp): Test negative values.
}

// Tests that we get the right number of edges with the HallEffectAndPosition
// sensor value.
TEST_F(PositionSensorSimTest, HallEffectAndPosition) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);
  sim.InitializeHallEffectAndPosition(-0.25, 0.0, 0.5);
  HallEffectAndPosition position;

  // Go over only the lower edge rising.
  sim.MoveTo(0.25);
  sim.GetSensorValues(&position);
  EXPECT_TRUE(position.current);
  EXPECT_DOUBLE_EQ(0.50, position.position);
  EXPECT_EQ(1, position.posedge_count);
  EXPECT_EQ(0.25, position.posedge_value);
  EXPECT_EQ(0, position.negedge_count);
  EXPECT_EQ(0, position.negedge_value);

  // Now, go over the upper edge, falling.
  sim.MoveTo(0.75);
  sim.GetSensorValues(&position);
  EXPECT_FALSE(position.current);
  EXPECT_DOUBLE_EQ(1.0, position.position);
  EXPECT_EQ(1, position.posedge_count);
  EXPECT_DOUBLE_EQ(0.25, position.posedge_value);
  EXPECT_EQ(1, position.negedge_count);
  EXPECT_DOUBLE_EQ(0.75, position.negedge_value);

  // Now, jump a whole cycle.
  sim.MoveTo(1.75);
  sim.GetSensorValues(&position);
  EXPECT_FALSE(position.current);
  EXPECT_DOUBLE_EQ(2.0, position.position);
  EXPECT_EQ(2, position.posedge_count);
  EXPECT_DOUBLE_EQ(1.25, position.posedge_value);
  EXPECT_EQ(2, position.negedge_count);
  EXPECT_DOUBLE_EQ(1.75, position.negedge_value);

  // Now, jump a whole cycle backwards.
  sim.MoveTo(0.75);
  sim.GetSensorValues(&position);
  EXPECT_FALSE(position.current);
  EXPECT_DOUBLE_EQ(1.0, position.position);
  EXPECT_EQ(3, position.posedge_count);
  EXPECT_DOUBLE_EQ(1.75, position.posedge_value);
  EXPECT_EQ(3, position.negedge_count);
  EXPECT_DOUBLE_EQ(1.25, position.negedge_value);

  // Now, go over the upper edge, rising.
  sim.MoveTo(0.25);
  sim.GetSensorValues(&position);
  EXPECT_TRUE(position.current);
  EXPECT_DOUBLE_EQ(0.5, position.position);
  EXPECT_EQ(4, position.posedge_count);
  EXPECT_DOUBLE_EQ(0.75, position.posedge_value);
  EXPECT_EQ(3, position.negedge_count);
  EXPECT_DOUBLE_EQ(1.25, position.negedge_value);

  // Now, go over the lower edge, falling.
  sim.MoveTo(-0.25);
  sim.GetSensorValues(&position);
  EXPECT_FALSE(position.current);
  EXPECT_DOUBLE_EQ(0.0, position.position);
  EXPECT_EQ(4, position.posedge_count);
  EXPECT_DOUBLE_EQ(0.75, position.posedge_value);
  EXPECT_EQ(4, position.negedge_count);
  EXPECT_DOUBLE_EQ(0.25, position.negedge_value);

  for (int i = 0; i < 10; ++i) {
    // Now, go over the lower edge, falling.
    sim.MoveTo(-0.25 - i * 1.0e-6);
    sim.GetSensorValues(&position);
    EXPECT_FALSE(position.current);
    EXPECT_NEAR(-i * 1.0e-6, position.position, 1e-8);
    EXPECT_EQ(4, position.posedge_count);
    EXPECT_DOUBLE_EQ(0.75, position.posedge_value);
    EXPECT_EQ(4, position.negedge_count);
    EXPECT_DOUBLE_EQ(0.25, position.negedge_value);
  }
}


}  // namespace control_loops
}  // namespace frc971
