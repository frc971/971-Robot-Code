#ifndef FRC971_ZEROING_AVERAGER_H_
#define FRC971_ZEROING_AVERAGER_H_

#include <algorithm>
#include <array>
#include <stdint.h>

#include "Eigen/Dense"
#include "glog/logging.h"

namespace frc971 {
namespace zeroing {

// Averages a set of given numbers. Numbers are given one at a time. Once full
// the average may be requested.
// TODO(james): Look at deduplicating this with some of the work in the
// MoveDetector.
template <typename Scalar, size_t num_samples, int rows_per_sample = 1>
class Averager {
 public:
  typedef Eigen::Matrix<Scalar, rows_per_sample, 1> Vector;
  Averager() {
    for (Vector &datum : data_) {
      datum.setZero();
    }
  }
  // Adds one data point to the set of data points to be averaged.
  // If more than "num_samples" samples are added, they will start overwriting
  // the oldest ones.
  void AddData(Scalar data) {
    CHECK_EQ(1, rows_per_sample);
    AddData(Vector(data));
  }
  void AddData(const Vector &data) {
    data_[data_point_index_] = data;
    num_data_points_ = std::min(num_samples, num_data_points_ + 1);
    data_point_index_ = (data_point_index_ + 1) % num_samples;
  }

  // Returns the average of the data points.
  Vector GetAverage() const {
    if (num_data_points_ == 0) {
      return Vector::Zero();
    }

    Vector average;
    average.setZero();
    for (const Vector &data : data_) {
      average += data;
    }
    return average / num_data_points_;
  }

  // Return the difference between the min and max values in the data buffer.
  Scalar GetRange() const {
    if (num_data_points_ == 0) {
      return 0.0;
    }
    Vector min_value;
    min_value.setConstant(std::numeric_limits<Scalar>::max());
    Vector max_value;
    max_value.setConstant(std::numeric_limits<Scalar>::lowest());
    // The array will always fill up starting at zero, so we can iterate from
    // zero safely.
    for (size_t ii = 0; ii < num_data_points_; ++ii) {
      const Vector &value = data_[ii];
      min_value = min_value.cwiseMin(value);
      max_value = max_value.cwiseMax(value);
    }
    return (max_value - min_value).maxCoeff();
  }

  void Reset() {
    num_data_points_ = 0;
    data_point_index_ = 0;
  }

  // Returns true when we've gathered num_samples data points.
  bool full() const { return num_data_points_ >= num_samples; };

  size_t size() const { return num_samples; }

 private:
  // Data points to be averaged.
  std::array<Vector, num_samples> data_;
  // Which data point in "data_" will be filled in next.
  size_t data_point_index_ = 0;
  // Number of data points added via AddData().
  size_t num_data_points_ = 0;
};

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_AVERAGER_H_
