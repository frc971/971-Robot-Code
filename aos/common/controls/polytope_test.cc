#include "aos/common/controls/polytope.h"

#include <vector>

#include "Eigen/Dense"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "aos/testing/test_logging.h"

namespace aos {
namespace controls {

class HPolytopeTest : public ::testing::Test {
 protected:
  HPolytope<2> Polytope1() {
    return HPolytope<2>{
        (Eigen::Matrix<double, 4, 2>() << 1, 0, -1, 0, 0, 1, 0, -1).finished(),
        (Eigen::Matrix<double, 4, 1>() << 12.0, 12.0, 12.0, 12.0).finished()};
  }
  HPolytope<2> Polytope2() {
    return HPolytope<2>{
        (Eigen::Matrix<double, 4, 2>() << 1, 1, -1, -1, 0, 1, 0, -1).finished(),
        (Eigen::Matrix<double, 4, 1>() << 12.0, 12.0, 12.0, 12.0).finished()};
  }
  HPolytope<1> Polytope3() {
    return HPolytope<1>{(Eigen::Matrix<double, 2, 1>() << 1, -0.5).finished(),
                        (Eigen::Matrix<double, 2, 1>() << 5.0, 2.0).finished()};
  }
  HPolytope<2> Polytope4() {
    return HPolytope<2>{
        (Eigen::Matrix<double, 4, 2>() << 1, 1, -1, -1, 1, -1, -1, 1)
            .finished(),
        (Eigen::Matrix<double, 4, 1>() << 2, -1, 2, -1).finished()};
  }
  HPolytope<2> Polytope5() {
    return HPolytope<2>{
        (Eigen::Matrix<double, 4, 2>() << 1, 1, -1, -1, 1, -1, -1, 1)
            .finished(),
        (Eigen::Matrix<double, 4, 1>() << 1.5, -0.5, 1.5, -0.5).finished()};
  }

  void SetUp() override {
    ::aos::testing::EnableTestLogging();
    ::aos::testing::ForcePrintLogsDuringTests();
    HPolytope<0>::Init();
  }

  template <typename T>
  ::std::vector<::std::vector<double>> MatrixToVectors(const T &matrix) {
    ::std::vector<::std::vector<double>> r;
    for (int i = 0; i < matrix.cols(); ++i) {
      ::std::vector<double> col;
      for (int j = 0; j < matrix.rows(); ++j) {
        col.emplace_back(matrix(j, i));
      }
      r.emplace_back(col);
    }
    return r;
  }

  template <typename T>
  ::std::vector<::testing::Matcher<::std::vector<double>>> MatrixToMatchers(
      const T &matrix) {
    ::std::vector<::testing::Matcher<::std::vector<double>>> r;
    for (int i = 0; i < matrix.cols(); ++i) {
      ::std::vector<::testing::Matcher<double>> col;
      for (int j = 0; j < matrix.rows(); ++j) {
        col.emplace_back(::testing::DoubleNear(matrix(j, i), 0.000001));
      }
      r.emplace_back(::testing::ElementsAreArray(col));
    }
    return r;
  }
};

// Tests that the vertices for various polytopes calculated from H and k are
// correct.
TEST_F(HPolytopeTest, CalculatedVertices) {
  EXPECT_THAT(MatrixToVectors(Polytope1().Vertices()),
              ::testing::UnorderedElementsAreArray(
                  MatrixToVectors((Eigen::Matrix<double, 2, 4>() << -12, -12,
                                   12, 12, -12, 12, 12, -12).finished())));
  EXPECT_THAT(MatrixToVectors(Polytope2().Vertices()),
              ::testing::UnorderedElementsAreArray(
                  MatrixToVectors((Eigen::Matrix<double, 2, 4>() << 24, 0, -24,
                                   0, -12, 12, 12, -12).finished())));
  EXPECT_THAT(MatrixToVectors(Polytope3().Vertices()),
              ::testing::UnorderedElementsAreArray(MatrixToVectors(
                  (Eigen::Matrix<double, 1, 2>() << 5, -4).finished())));
  EXPECT_THAT(MatrixToVectors(Polytope4().Vertices()),
              ::testing::UnorderedElementsAreArray(
                  MatrixToVectors((Eigen::Matrix<double, 2, 4>() << 1, 1.5, 1.5,
                                   2, 0, -0.5, 0.5, 0).finished())));
  EXPECT_THAT(MatrixToVectors(Polytope5().Vertices()),
              ::testing::UnorderedElementsAreArray(
                  MatrixToVectors((Eigen::Matrix<double, 2, 4>() << 0.5, 1, 1.5,
                                   1, 0, 0.5, 0, -0.5).finished())));
}

}  // namespace controls
}  // namespace aos
