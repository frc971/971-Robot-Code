#include <unistd.h>

#include "frc971/control_loops/voltage_cap/voltage_cap.h"

#include "gtest/gtest.h"

#include "aos/common/queue.h"
#include "aos/testing/test_shm.h"

namespace frc971 {
namespace control_loops {
namespace testing {

class VoltageTest : public ::testing::Test {
 protected:
  // Bring up and down Core.
  ::aos::testing::TestSharedMemory my_shm_;
};

// Tests that voltage inputs return the same if inside the box.
TEST_F(VoltageTest, BasicVoltage12) {
  double voltage_one, voltage_two;
  VoltageCap(12.0, 11.3, 2.6, &voltage_one, &voltage_two);
  EXPECT_EQ(11.3, voltage_one);
  EXPECT_EQ(2.6, voltage_two);
}
// Tests that voltage inputs in the 4th quadrant both get capped to their
// maximum.
TEST_F(VoltageTest, QuadrantFourNoIntersect12) {
  double voltage_one, voltage_two;
  VoltageCap(12.0, -50.0, 50, &voltage_one, &voltage_two);
  EXPECT_EQ(-12.0, voltage_one);
  EXPECT_EQ(12.0, voltage_two);
}

// Tests if the difference between two voltages is more than 24.0v then default
// to the most outputable voltage in that direction.
TEST_F(VoltageTest, LargeDifference12) {
  double voltage_one, voltage_two;
  VoltageCap(12.0, -13.0, 13.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-12.0, voltage_one);  // just off bottom right corner
  EXPECT_EQ(12.0, voltage_two);

  VoltageCap(12.0, 13.0, -13.0, &voltage_one, &voltage_two);
  EXPECT_EQ(12.0, voltage_one);  // just off top left corner
  EXPECT_EQ(-12.0, voltage_two);

  VoltageCap(12.0, 10.0, 36.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-12.0, voltage_one);  // 1st quadrant line just off bottom right
  EXPECT_EQ(12.0, voltage_two);

  VoltageCap(12.0, 10.0, -36.0, &voltage_one, &voltage_two);
  EXPECT_EQ(12.0, voltage_one);  // 3rd quadrant line just off top left
  EXPECT_EQ(-12.0, voltage_two);
}

// Tests that the 45degree angle line intersects the box and returns a value
// within the box.
TEST_F(VoltageTest, QuadrantOneIntersect12) {
  double voltage_one, voltage_two;
  VoltageCap(12.0, 50.0, 50.0, &voltage_one, &voltage_two);
  EXPECT_EQ(12.0, voltage_one);  // 45degree angle from origin
  EXPECT_EQ(12.0, voltage_two);

  voltage_one = 0.0;

  VoltageCap(12.0, 13.0, 11.0, &voltage_one, &voltage_two);
  EXPECT_EQ(12.0, voltage_one);
  EXPECT_EQ(10.0, voltage_two);
}
TEST_F(VoltageTest, QuadrantTwoIntersect12) {
  double voltage_one, voltage_two;
  VoltageCap(12.0, 13.0, -2.0, &voltage_one, &voltage_two);
  EXPECT_EQ(12.0, voltage_one);
  EXPECT_EQ(-3.0, voltage_two);

  VoltageCap(12.0, 2.0, -13.0, &voltage_one, &voltage_two);
  EXPECT_EQ(3.0, voltage_one);
  EXPECT_EQ(-12.0, voltage_two);
}
TEST_F(VoltageTest, QuadrantThreeIntersect12) {
  double voltage_one, voltage_two;
  VoltageCap(12.0, -50.0, -50.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-12.0, voltage_one);  // 45degree angle from origin
  EXPECT_EQ(-12.0, voltage_two);

  voltage_one = 0.0;

  VoltageCap(12.0, -13.0, -11.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-12.0, voltage_one);
  EXPECT_EQ(-10.0, voltage_two);
}
TEST_F(VoltageTest, QuadrantFourIntersect12) {
  double voltage_one, voltage_two;
  VoltageCap(12.0, -13.0, 2.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-12.0, voltage_one);
  EXPECT_EQ(3.0, voltage_two);

  VoltageCap(12.0, -2.0, 13.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-3.0, voltage_one);
  EXPECT_EQ(12.0, voltage_two);
}

// Tests whether cross quadrants works (also supplies additional points to
// test).
TEST_F(VoltageTest, QuadrantOneToTwo12) {
  // Point in Quadrant 1 intersects box in Quadrant 2.
  double voltage_one, voltage_two;
  VoltageCap(12.0, 33.0, 11.0, &voltage_one, &voltage_two);
  EXPECT_EQ(12.0, voltage_one);
  EXPECT_EQ(-10.0, voltage_two);
}
TEST_F(VoltageTest, QuadrantOneToFour12){
  // Point in Quadrant 1 intersects box in Quadrant 4.
  double voltage_one, voltage_two;
  VoltageCap(12.0, 11.0, 33.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-10.0, voltage_one);
  EXPECT_EQ(12.0, voltage_two);
}
TEST_F(VoltageTest, QuadrantThreeToTwo12) {
  // Point in Quadrant 3 intersects box in Quadrant 2.
  double voltage_one, voltage_two;
  VoltageCap(12.0, -11.0, -33.0, &voltage_one, &voltage_two);
  EXPECT_EQ(10.0, voltage_one);
  EXPECT_EQ(-12.0, voltage_two);
}
TEST_F(VoltageTest, QuadrantThreeToFour12) {
  // Point in Quadrant 3 intersects box in Quadrant 4.
  double voltage_one, voltage_two;
  VoltageCap(12.0, -33.0, -11.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-12.0, voltage_one);
  EXPECT_EQ(10.0, voltage_two);
}

// Tests that voltage inputs return the same if inside the box.
TEST_F(VoltageTest, BasicVoltage6) {
  double voltage_one, voltage_two;
  VoltageCap(6.0, 5.3, 2.6, &voltage_one, &voltage_two);
  EXPECT_EQ(5.3, voltage_one);
  EXPECT_EQ(2.6, voltage_two);
}
// Tests that voltage inputs in the 4th quadrant both get capped to their
// maximum.
TEST_F(VoltageTest, QuadrantFourNoIntersect6) {
  double voltage_one, voltage_two;
  VoltageCap(6.0, -50.0, 50, &voltage_one, &voltage_two);
  EXPECT_EQ(-6.0, voltage_one);
  EXPECT_EQ(6.0, voltage_two);
}

// Tests if the difference between two voltages is more than 12.0v then default
// to the most outputable voltage in that direction.
TEST_F(VoltageTest, LargeDifference6) {
  double voltage_one, voltage_two;
  VoltageCap(6.0, -13.0, 13.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-6.0, voltage_one);  // just off bottom right corner
  EXPECT_EQ(6.0, voltage_two);

  VoltageCap(6.0, 13.0, -13.0, &voltage_one, &voltage_two);
  EXPECT_EQ(6.0, voltage_one);  // just off top left corner
  EXPECT_EQ(-6.0, voltage_two);

  VoltageCap(6.0, 10.0, 36.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-6.0, voltage_one);  // 1st quadrant line just off bottom right
  EXPECT_EQ(6.0, voltage_two);

  VoltageCap(6.0, 10.0, -36.0, &voltage_one, &voltage_two);
  EXPECT_EQ(6.0, voltage_one);  // 3rd quadrant line just off top left
  EXPECT_EQ(-6.0, voltage_two);
}

// Tests that the 45degree angle line intersects the box and returns a value
// within the box
TEST_F(VoltageTest, QuadrantOneIntersect6) {
  double voltage_one, voltage_two;
  VoltageCap(6.0, 50.0, 50.0, &voltage_one, &voltage_two);
  EXPECT_EQ(6.0, voltage_one);  // 45degree angle from origin
  EXPECT_EQ(6.0, voltage_two);

  voltage_one = 0.0;

  VoltageCap(6.0, 13.0, 11.0, &voltage_one, &voltage_two);
  EXPECT_EQ(6.0, voltage_one);
  EXPECT_EQ(4.0, voltage_two);
}
TEST_F(VoltageTest, QuadrantTwoIntersect6) {
  double voltage_one, voltage_two;
  VoltageCap(6.0, 9.0, -2.0, &voltage_one, &voltage_two);
  EXPECT_EQ(6.0, voltage_one);
  EXPECT_EQ(-5.0, voltage_two);

  VoltageCap(6.0, 2.0, -9.0, &voltage_one, &voltage_two);
  EXPECT_EQ(5.0, voltage_one);
  EXPECT_EQ(-6.0, voltage_two);
}
TEST_F(VoltageTest, QuadrantThreeIntersect6) {
  double voltage_one, voltage_two;
  VoltageCap(6.0, -50.0, -50.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-6.0, voltage_one);  // 45degree angle from origin
  EXPECT_EQ(-6.0, voltage_two);

  voltage_one = 0.0;

  VoltageCap(6.0, -13.0, -11.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-6.0, voltage_one);
  EXPECT_EQ(-4.0, voltage_two);
}
TEST_F(VoltageTest, QuadrantFourIntersect6) {
  double voltage_one, voltage_two;
  VoltageCap(6.0, -9.0, 2.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-6.0, voltage_one);
  EXPECT_EQ(5.0, voltage_two);

  VoltageCap(6.0, -2.0, 9.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-5.0, voltage_one);
  EXPECT_EQ(6.0, voltage_two);
}

// Tests whether cross quadrants works (also supplies additional points to
// test).
TEST_F(VoltageTest, QuadrantOneToTwo6) {
  // Point in Quadrant 1 intersects box in Quadrant 2.
  double voltage_one, voltage_two;
  VoltageCap(6.0, 33.0, 22.0, &voltage_one, &voltage_two);
  EXPECT_EQ(6.0, voltage_one);
  EXPECT_EQ(-5.0, voltage_two);
}
TEST_F(VoltageTest, QuadrantOneToFour6){
  // Point in Quadrant 1 intersects box in Quadrant 4.
  double voltage_one, voltage_two;
  VoltageCap(6.0, 22.0, 33.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-5.0, voltage_one);
  EXPECT_EQ(6.0, voltage_two);
}
TEST_F(VoltageTest, QuadrantThreeToTw6) {
  // Point in Quadrant 3 intersects box in Quadrant 2.
  double voltage_one, voltage_two;
  VoltageCap(6.0, -22.0, -33.0, &voltage_one, &voltage_two);
  EXPECT_EQ(5.0, voltage_one);
  EXPECT_EQ(-6.0, voltage_two);
}
TEST_F(VoltageTest, QuadrantThreeToFour6) {
  // Point in Quadrant 3 intersects box in Quadrant 4.
  double voltage_one, voltage_two;
  VoltageCap(6.0, -33.0, -22.0, &voltage_one, &voltage_two);
  EXPECT_EQ(-6.0, voltage_one);
  EXPECT_EQ(5.0, voltage_two);
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
