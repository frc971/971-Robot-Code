#include "y2017/control_loops/superstructure/vision_distance_average.h"

#include "gtest/gtest.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {

class VisionDistanceAverageTest : public ::testing::Test {
 public:
  ::aos::monotonic_clock::time_point tick_time() {
    current_time_ += std::chrono::milliseconds(60);
    return current_time_;
  }

  VisionDistanceAverage* average() {
    return &average_;
  }

  void TickInvalid() {
    vision::VisionStatus status;
    status.image_valid = false;

    average_.Tick(tick_time(), &status);
  }

  void TickValid(double distance) {
    vision::VisionStatus status;
    status.image_valid = true;
    status.distance = distance;

    average_.Tick(tick_time(), &status);
  }

  void TickNullptr() {
    average_.Tick(tick_time(), nullptr); 
  }

 private:
  ::aos::monotonic_clock::time_point current_time_ =
      ::aos::monotonic_clock::epoch() + std::chrono::seconds(300);

  VisionDistanceAverage average_;
};

TEST_F(VisionDistanceAverageTest, AverageTest) {
  EXPECT_FALSE(average()->Valid());
  for (int i = 0; i < 100; ++i) {
    TickValid(2.0 + i);
  }
  EXPECT_TRUE(average()->Valid());
  EXPECT_EQ(89.0, average()->Get());
}

TEST_F(VisionDistanceAverageTest, AverageDropCount) {
  EXPECT_FALSE(average()->Valid());
  for (int i = 0; i < 100; ++i) {
    TickValid(2.0 + i);
  }
  EXPECT_TRUE(average()->Valid());
  for (int i = 0; i < 100; ++i) {
    TickInvalid();
  }
  TickNullptr();
  EXPECT_FALSE(average()->Valid());
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
