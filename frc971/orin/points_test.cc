#include "frc971/orin/points.h"

#include <random>

#include "gtest/gtest.h"

#include "aos/testing/random_seed.h"

namespace frc971::apriltag::testing {

// Tests that QuadBoundaryPoint doesn't corrupt data.
TEST(QuadBoundaryPoint, MasksWork) {
  std::mt19937 generator(aos::testing::RandomSeed());
  std::uniform_int_distribution<uint32_t> random_rep_scalar(0, 0xfffff);
  std::uniform_int_distribution<uint32_t> random_point_scalar(0, 0x3ff);
  std::uniform_int_distribution<uint32_t> random_dxy_scalar(0, 3);
  std::uniform_int_distribution<uint32_t> random_bool(0, 1);

  QuadBoundaryPoint point;

  EXPECT_EQ(point.key, 0);

  for (int i = 0; i < 25; ++i) {
    const uint32_t rep0 = random_rep_scalar(generator);
    for (int j = 0; j < 25; ++j) {
      const uint32_t rep1 = random_rep_scalar(generator);
      for (int k = 0; k < 25; ++k) {
        const uint32_t x = random_point_scalar(generator);
        const uint32_t y = random_point_scalar(generator);
        for (int k = 0; k < 25; ++k) {
          const uint32_t dxy = random_dxy_scalar(generator);
          for (int m = 0; m < 2; ++m) {
            const bool black_to_white = random_bool(generator) == 1;

            if (point.rep0() != rep0) {
              point.set_rep0(rep0);
            }

            if (point.rep1() != rep1) {
              point.set_rep1(rep1);
            }

            if (point.base_x() != x || point.base_y() != y) {
              point.set_base_xy(x, y);
            }

            switch (dxy) {
              case 0:
                if (point.dx() != 1 || point.dy() != 0) {
                  point.set_dxy(dxy);
                }
                break;
              case 1:
                if (point.dx() != 1 || point.dy() != 1) {
                  point.set_dxy(dxy);
                }
                break;
              case 2:
                if (point.dx() != 0 || point.dy() != 1) {
                  point.set_dxy(dxy);
                }
                break;
              case 3:
                if (point.dx() != -1 || point.dy() != 1) {
                  point.set_dxy(dxy);
                }
                break;
            }

            if (black_to_white != point.black_to_white()) {
              point.set_black_to_white(black_to_white);
            }

            EXPECT_EQ(point.rep0(), rep0);
            EXPECT_EQ(point.rep1(), rep1);
            EXPECT_EQ(point.base_x(), x);
            EXPECT_EQ(point.base_y(), y);
            switch (dxy) {
              case 0:
                EXPECT_EQ(point.dx(), 1);
                EXPECT_EQ(point.dy(), 0);
                break;
              case 1:
                EXPECT_EQ(point.dx(), 1);
                EXPECT_EQ(point.dy(), 1);
                break;
              case 2:
                EXPECT_EQ(point.dx(), 0);
                EXPECT_EQ(point.dy(), 1);
                break;
              case 3:
                EXPECT_EQ(point.dx(), -1);
                EXPECT_EQ(point.dy(), 1);
                break;
            }

            EXPECT_EQ(point.x(), x * 2 + point.dx());
            EXPECT_EQ(point.y(), y * 2 + point.dy());

            EXPECT_EQ(point.black_to_white(), black_to_white);
          }
        }
      }
    }
  }
}

// Tests that IndexPoint doesn't corrupt anything
TEST(IndexPoint, MasksWork) {
  std::mt19937 generator(
      aos::testing::RandomSeed());  // random_uint32(generator)
  std::uniform_int_distribution<uint32_t> random_blob_index(0, 0xfff);
  std::uniform_int_distribution<uint32_t> random_theta(0, 0xfffffff);
  std::uniform_int_distribution<uint32_t> random_point_scalar(0, 0x3ff);
  std::uniform_int_distribution<uint32_t> random_dxy_scalar(0, 3);
  std::uniform_int_distribution<uint32_t> random_bool(0, 1);

  IndexPoint point;

  for (int i = 0; i < 25; i++) {
    const uint32_t blob_index = random_blob_index(generator);
    for (int j = 0; j < 25; j++) {
      const uint32_t theta = random_theta(generator);
      for (int k = 0; k < 25; ++k) {
        const uint32_t x = random_point_scalar(generator);
        const uint32_t y = random_point_scalar(generator);
        for (int k = 0; k < 25; ++k) {
          const uint32_t dxy = random_dxy_scalar(generator);
          for (int m = 0; m < 2; ++m) {
            const bool black_to_white = random_bool(generator) == 1;

            if (point.blob_index() != blob_index) {
              point.set_blob_index(blob_index);
            }

            if (point.theta() != theta) {
              point.set_theta(theta);
            }

            if (point.base_x() != x || point.base_y() != y) {
              point.set_base_xy(x, y);
            }

            switch (dxy) {
              case 0:
                if (point.dx() != 1 || point.dy() != 0) {
                  point.set_dxy(dxy);
                }
                break;
              case 1:
                if (point.dx() != 1 || point.dy() != 1) {
                  point.set_dxy(dxy);
                }
                break;
              case 2:
                if (point.dx() != 0 || point.dy() != 1) {
                  point.set_dxy(dxy);
                }
                break;
              case 3:
                if (point.dx() != -1 || point.dy() != 1) {
                  point.set_dxy(dxy);
                }
                break;
            }

            if (black_to_white != point.black_to_white()) {
              point.set_black_to_white(black_to_white);
            }

            EXPECT_EQ(point.blob_index(), blob_index);
            EXPECT_EQ(point.theta(), theta);
            EXPECT_EQ(point.base_x(), x);
            EXPECT_EQ(point.base_y(), y);

            switch (dxy) {
              case 0:
                EXPECT_EQ(point.dx(), 1);
                EXPECT_EQ(point.dy(), 0);
                break;
              case 1:
                EXPECT_EQ(point.dx(), 1);
                EXPECT_EQ(point.dy(), 1);
                break;
              case 2:
                EXPECT_EQ(point.dx(), 0);
                EXPECT_EQ(point.dy(), 1);
                break;
              case 3:
                EXPECT_EQ(point.dx(), -1);
                EXPECT_EQ(point.dy(), 1);
                break;
            }
            EXPECT_EQ(point.x(), x * 2 + point.dx());
            EXPECT_EQ(point.y(), y * 2 + point.dy());

            EXPECT_EQ(point.black_to_white(), black_to_white);
          }
        }
      }
    }
  }
}
}  // namespace frc971::apriltag::testing
