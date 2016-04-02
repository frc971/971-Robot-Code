#ifndef FRC971_ZEROING_AVERAGER_H_
#define FRC971_ZEROING_AVERAGER_H_

#include <algorithm>
#include <array>
#include <stdint.h>

namespace frc971 {
namespace zeroing {

// Averages a set of given numbers. Numbers are given one at a time. Once full
// the average may be requested.
template <typename data_type, size_t data_size>
class Averager {
 public:
  // Adds one data point to the set of data points to be averaged.
  // If more than "data_size" samples are added, they will start overwriting
  // the oldest ones.
  void AddData(data_type data) {
    data_[data_point_index_] = data;
    num_data_points_ = ::std::min(data_size, num_data_points_ + 1);
    data_point_index_ = (data_point_index_ + 1) % data_size;
  }

  // Returns the average of the data points.
  data_type GetAverage() const {
    // TODO(phil): What do we want to do without any elements?
    if (num_data_points_ == 0) {
      return 0;
    }

    data_type average = 0;
    for (data_type data : data_) {
      average += data;
    }
    return average / num_data_points_;
  }

  // Returns true when we've gathered data_size data points.
  bool full() const { return num_data_points_ >= data_size; };

  size_t size() const { return data_size; }

 private:
  // Data points to be averaged.
  ::std::array<data_type, data_size> data_;
  // Which data point in "data_" will be filled in next.
  size_t data_point_index_ = 0;
  // Number of data points added via AddData().
  size_t num_data_points_ = 0;
};

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_AVERAGER_H_
