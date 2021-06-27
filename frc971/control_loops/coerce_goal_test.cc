#include "frc971/control_loops/coerce_goal.h"

#include <unistd.h>

#include "frc971/control_loops/polytope.h"
#include "gtest/gtest.h"

namespace frc971 {
namespace control_loops {

namespace {

frc971::controls::HVPolytope<2, 4, 4> MakeBox(double x1_min, double x1_max,
                                              double x2_min, double x2_max) {
  Eigen::Matrix<double, 4, 2> box_H;
  box_H << /*[[*/ 1.0, 0.0 /*]*/,
      /*[*/ -1.0, 0.0 /*]*/,
      /*[*/ 0.0, 1.0 /*]*/,
      /*[*/ 0.0, -1.0 /*]]*/;
  Eigen::Matrix<double, 4, 1> box_k;
  box_k << /*[[*/ x1_max /*]*/,
      /*[*/ -x1_min /*]*/,
      /*[*/ x2_max /*]*/,
      /*[*/ -x2_min /*]]*/;
  frc971::controls::HPolytope<2> t_poly(box_H, box_k);
  return frc971::controls::HVPolytope<2, 4, 4>(t_poly.H(), t_poly.k(),
                                               t_poly.Vertices());
}
}  // namespace

class CoerceGoalTest : public ::testing::Test {
 public:
  void SetUp() override { frc971::controls::HPolytope<2>::Init(); }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(CoerceGoalTest, Inside) {
  frc971::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << /*[[*/ 1, -1 /*]]*/;

  Eigen::Matrix<double, 2, 1> R;
  R << /*[[*/ 1.5, 1.5 /*]]*/;

  Eigen::Matrix<double, 2, 1> output =
      frc971::control_loops::CoerceGoal<double>(box, K, 0, R);

  EXPECT_EQ(R(0, 0), output(0, 0));
  EXPECT_EQ(R(1, 0), output(1, 0));
}

TEST_F(CoerceGoalTest, LineOutside) {
  frc971::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  // Make a line equivalent to y = -x, which does not pass through the box and
  // is nearest the box at (1, 1).
  Eigen::Matrix<double, 1, 2> K;
  K << 1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << /*[[*/ 0.0, 0.0 /*]]*/;

  Eigen::Matrix<double, 2, 1> output =
      frc971::control_loops::CoerceGoal<double>(box, K, 0, R);

  EXPECT_EQ(1.0, output(0, 0));
  EXPECT_EQ(1.0, output(1, 0));

  // Test the same line, but on the other side of the box, where the (2, 2)
  // vertex will be closest.
  output = frc971::control_loops::CoerceGoal<double>(box, K, 5, R);
  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, GoalOutsideLineInsideThroughOrigin) {
  frc971::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, -1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      frc971::control_loops::CoerceGoal<double>(box, K, 0, R);

  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, GoalOutsideLineNotThroughOrigin) {
  frc971::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 0, 3;

  Eigen::Matrix<double, 2, 1> output =
      frc971::control_loops::CoerceGoal<double>(box, K, 3, R);

  EXPECT_EQ(1.0, output(0, 0));
  EXPECT_DOUBLE_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, GoalOutsideLineThroughVertex) {
  frc971::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, -1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      frc971::control_loops::CoerceGoal<double>(box, K, 1, R);

  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(1.0, output(1, 0));
}

TEST_F(CoerceGoalTest, LineAndGoalOutside) {
  frc971::controls::HVPolytope<2, 4, 4> box = MakeBox(3, 4, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, -1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      frc971::control_loops::CoerceGoal<double>(box, K, 0, R);

  EXPECT_EQ(3.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, LineThroughEdgeOfBox) {
  frc971::controls::HVPolytope<2, 4, 4> box = MakeBox(0, 4, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << -1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      frc971::control_loops::CoerceGoal<double>(box, K, 0, R);

  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, PerpendicularLine) {
  frc971::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      frc971::control_loops::CoerceGoal<double>(box, K, 0, R);

  EXPECT_EQ(1.0, output(0, 0));
  EXPECT_EQ(1.0, output(1, 0));
}

TEST_F(CoerceGoalTest, WithinRegion) {
  const auto upoly = MakeBox(-12.0, 12.0, -12.0, 12.0);
  Eigen::Matrix<double, 1, 2> k;
  k << 2, 2;

  Eigen::Matrix<double, 2, 1> goal;
  goal << -2, 2;

  auto result = CoerceGoal<double>(upoly, k, 0, goal);

  EXPECT_EQ(result(0, 0), goal(0, 0));
  EXPECT_EQ(result(1, 0), goal(1, 0));
}

TEST_F(CoerceGoalTest, VerticalLine) {
  const auto upoly = MakeBox(-12.0, 12.0, -12.0, 12.0);
  Eigen::Matrix<double, 1, 2> k;
  k << 2, 0;

  Eigen::Matrix<double, 2, 1> goal;
  goal << 0, 13;

  auto result = CoerceGoal<double>(upoly, k, 0, goal);

  EXPECT_EQ(result(0, 0), 0);
  EXPECT_EQ(result(1, 0), 12);
}

TEST_F(CoerceGoalTest, HorizontalLine) {
  const auto upoly = MakeBox(-12.0, 12.0, -12.0, 12.0);
  Eigen::Matrix<double, 1, 2> k;
  k << 0, 2;

  Eigen::Matrix<double, 2, 1> goal;
  goal << 13, 2;

  auto result = CoerceGoal<double>(upoly, k, 0, goal);

  EXPECT_EQ(result(0, 0), 12);
  EXPECT_EQ(result(1, 0), 0);
}

}  // namespace control_loops
}  // namespace frc971
