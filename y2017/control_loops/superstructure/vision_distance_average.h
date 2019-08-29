#ifndef Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_VISION_DISTANCE_AVERAGE_H_
#define Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_VISION_DISTANCE_AVERAGE_H_

#include <stdint.h>
#include <chrono>

#include "aos/containers/ring_buffer.h"
#include "aos/time/time.h"
#include "y2017/vision/vision_generated.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {

namespace chrono = ::std::chrono;

// Averages out the distance from the vision system by a factor of 25.
class VisionDistanceAverage {
 public:
  // Call on every tick of the control loop to update the state.
  void Tick(::aos::monotonic_clock::time_point monotonic_now,
            const vision::VisionStatus *vision_status) {
    constexpr chrono::seconds kCullTimeDelta = chrono::seconds(2);
    auto cull_time = monotonic_now - kCullTimeDelta;
    // Delete data that is older than kCullTimeDelta seconds ago.
    while (data_.size() > 0 && data_[0].time < cull_time) {
      data_.Shift();
      cached_value_ = ComputeValue();
    }

    if (vision_status != nullptr && vision_status->image_valid()) {
      data_.Push({monotonic_now, vision_status->distance()});
      cached_value_ = ComputeValue();
    }
  }

  // Does not update while not seeing new images.
  double Get() { return cached_value_; }

  // Valid gives a sense of how recent the data is.
  bool Valid() { return data_.size() > 4; }

  // Clears all the saved samples.
  void Reset() { data_.Reset(); }

 private:
  double ComputeValue() {
    double result = 0.0;
    for (size_t i = 0; i < data_.size(); ++i) {
      result += data_[i].value;
    }
    return result / data_.size();
  }

  struct DistanceEvent {
    ::aos::monotonic_clock::time_point time;
    double value;
  };

  double cached_value_ = 0.0;
  ::aos::RingBuffer<DistanceEvent, 25> data_;
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017

#endif  // Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_VISION_DISTANCE_AVERAGE_H_
