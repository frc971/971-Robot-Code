#include "frc971/control_loops/elevator_and_pivot/inverse_kinematics.h"

#include "gtest/gtest.h"

#include "inverse_kinematics.h"

namespace frc971::control_loops::elevator_pivot::testing {
TEST(IkTest, ElevatorPivotIkPosPos) {
  Cartesian coords{std::sqrt(3.0) / 2.0, 2.0};
  auto result = solve_ik(coords, 1.0);
  EXPECT_NEAR(result.first.theta, std::numbers::pi / 3.0, 1e-9);
  EXPECT_EQ(result.first.distance, 1.5);
  EXPECT_NEAR(result.second.theta, -2.0 * std::numbers::pi / 3.0, 1e-9);
  EXPECT_EQ(result.second.distance, 2.5);
}

TEST(IkTest, ElevatorPivotIkNegPos) {
  Cartesian coords{-std::sqrt(3.0) / 2.0, 2.0};
  auto result = solve_ik(coords, 1.0);
  EXPECT_NEAR(result.first.theta, -std::numbers::pi / 3.0, 1e-9);
  EXPECT_EQ(result.first.distance, 1.5);
  EXPECT_NEAR(result.second.theta, 2.0 * std::numbers::pi / 3.0, 1e-9);
  EXPECT_EQ(result.second.distance, 2.5);
}

TEST(IkTest, ElevatorPivotPointAngleFar) {
  // an unreachable goal so it's points are along a line
  Cartesian coords{10.0, 10.0};
  auto result = solve_ik({coords, -std::numbers::pi / 4.0}, 1.0);
  EXPECT_NEAR(result.first.theta, std::numbers::pi / 2.0, 1e-9);
  EXPECT_NEAR(result.first.distance, 1.0, 1e-9);
  EXPECT_NEAR(result.second.theta, -std::numbers::pi / 2.0, 1e-9);
  EXPECT_NEAR(result.second.distance, -1.0, 1e-9);
}

TEST(IkTest, ElevatorPivotPointAngleFarReverse) {
  // an unreachable goal so it's points are along a line but with negative x
  Cartesian coords{-10.0, 10.0};
  auto result = solve_ik({coords, std::numbers::pi / 4.0}, 1.0);
  EXPECT_NEAR(result.first.theta, -std::numbers::pi / 2.0, 1e-9);
  EXPECT_NEAR(result.first.distance, 1.0, 1e-9);
  EXPECT_NEAR(result.second.theta, std::numbers::pi / 2.0, 1e-9);
  EXPECT_NEAR(result.second.distance, -1.0, 1e-9);
}

TEST(IkTest, ElevatorPivotPointAngleFarOffset) {
  // an unreachable goal so it's points are along a line but are offset up in y
  {
    Cartesian coords{10.0, 11.0};
    auto result = solve_ik({coords, -5.0 * std::numbers::pi / 4.0}, 1.0);
    EXPECT_NEAR(result.first.theta, std::numbers::pi / 2.0, 1e-9);
    EXPECT_NEAR(result.first.distance, 2.0, 1e-9);
    EXPECT_NEAR(result.second.theta, -std::numbers::pi / 2.0, 1e-9);
    EXPECT_NEAR(result.second.distance, 0.0, 1e-9);
  }
  // offset in x
  {
    Cartesian coords{9.0, 10.0};
    auto result = solve_ik({coords, -5.0 * std::numbers::pi / 4.0}, 1.0);
    EXPECT_NEAR(result.first.theta, std::numbers::pi / 2.0, 1e-9);
    EXPECT_NEAR(result.first.distance, 2.0, 1e-9);
    EXPECT_NEAR(result.second.theta, -std::numbers::pi / 2.0, 1e-9);
    EXPECT_NEAR(result.second.distance, 0.0, 1e-9);
  }
}

TEST(IkTest, ElevatorPivotPointAngleNot45) {
  // an unreachable goal with a non 45 degree line
  Cartesian coords{4.0, 10.0};
  auto result = solve_ik({coords, -std::numbers::pi / 3.0}, 2.0);
  EXPECT_NEAR(result.first.theta, std::numbers::pi / 2.0, 1e-9);
  // solve for the other point using the parametric equation with x = 2 and -2
  // x = 4 + t * cos(60 deg)
  // y = 10 + t * sin(60 deg)
  EXPECT_NEAR(result.first.distance, 10 - 4.0 * std::sqrt(3.0) / 2.0, 1e-9);
  EXPECT_NEAR(result.second.theta, -std::numbers::pi / 2.0, 1e-9);
  EXPECT_NEAR(result.second.distance, 10 - 12.0 * sqrt(3.0) / 2.0, 1e-9);
}

TEST(IkTest, ElevatorPivotPointAngleClose) {
  // normal IK since it's close(test same as above)
  Cartesian coords{-std::sqrt(3.0) / 2.0, 2.0};
  auto result = solve_ik({coords, -std::numbers::pi / 4.0}, 1.0);
  EXPECT_NEAR(result.first.theta, -std::numbers::pi / 3.0, 1e-9);
  EXPECT_EQ(result.first.distance, 1.5);
  EXPECT_NEAR(result.second.theta, 2.0 * std::numbers::pi / 3.0, 1e-9);
  EXPECT_EQ(result.second.distance, 2.5);
}

TEST(IkTest, FromSetpoint) {
  {
    auto result =
        from_setpoint(2.0, std::numbers::pi / 6.0, 3.0, std::numbers::pi / 4.0);
    EXPECT_EQ(result.point.x, 3.0 * std::sin(std::numbers::pi / 6.0));
    EXPECT_EQ(result.point.y, 2.0 + 3.0 * std::cos(std::numbers::pi / 6.0));
    EXPECT_EQ(result.theta, 11.0 * std::numbers::pi / 12.0);
  }
  {
    auto result = from_setpoint(2.0, -std::numbers::pi / 6.0, 3.0,
                                -std::numbers::pi / 4.0);
    EXPECT_EQ(result.point.x, -3.0 * std::sin(std::numbers::pi / 6.0));
    EXPECT_EQ(result.point.y, 2.0 + 3.0 * std::cos(std::numbers::pi / 6.0));
    EXPECT_NEAR(result.theta, std::numbers::pi / 12.0, 1e-9);
  }
}
}  // namespace frc971::control_loops::elevator_pivot::testing
