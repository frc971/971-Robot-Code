#ifndef AOS_VISION_BLOB_THRESHOLD_H_
#define AOS_VISION_BLOB_THRESHOLD_H_

#include <array>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

#include "aos/vision/blob/range_image.h"
#include "aos/vision/image/image_types.h"

namespace aos {
namespace vision {
namespace threshold_internal {

// Performs thresholding in a given region using a function which determines
// whether a given point is in or out of the region.
//
// fn must return a bool when called with two integers (x, y).
template <typename PointTestFn>
RangeImage ThresholdPointsWithFunction(ImageFormat fmt, PointTestFn &&fn) {
  static_assert(
      std::is_convertible<PointTestFn, std::function<bool(int, int)>>::value,
      "Invalid threshold function");
  std::vector<std::vector<ImageRange>> result;
  result.reserve(fmt.h);
  // Iterate through each row.
  for (int y = 0; y < fmt.h; ++y) {
    // Whether we're currently in a range.
    bool in_range = false;
    int current_range_start = -1;
    std::vector<ImageRange> current_row_ranges;
    // Iterate through each pixel.
    for (int x = 0; x < fmt.w; ++x) {
      if (fn(x, y) != in_range) {
        if (in_range) {
          current_row_ranges.emplace_back(ImageRange(current_range_start, x));
        } else {
          current_range_start = x;
        }
        in_range = !in_range;
      }
    }
    if (in_range) {
      current_row_ranges.emplace_back(ImageRange(current_range_start, fmt.w));
    }
    result.push_back(current_row_ranges);
  }
  return RangeImage(0, std::move(result));
}

}  // namespace threshold_internal

// Thresholds an image using a function which determines whether a given pixel
// value is in or out of the region.
//
// fn must return a bool when called with a PixelRef.
template <typename ThresholdFn>
RangeImage ThresholdImageWithFunction(const ImagePtr &img, ThresholdFn &&fn) {
  static_assert(
      std::is_convertible<ThresholdFn, std::function<bool(PixelRef)>>::value,
      "Invalid threshold function");
  return threshold_internal::ThresholdPointsWithFunction(
      img.fmt(), [&](int x, int y) { return fn(img.get_px(x, y)); });
}

// Thresholds an image in YUYV format, selecting pixels with a Y (luma) greater
// than value.
//
// This is implemented via a simple function that pulls out the Y values and
// compares them each. It mostly exists for tests to compare against
// FastYuyvYThreshold, because it's obviously correct.
inline RangeImage SlowYuyvYThreshold(ImageFormat fmt, const char *data,
                                     uint8_t value) {
  return threshold_internal::ThresholdPointsWithFunction(
      fmt, [&](int x, int y) {
        uint8_t v = data[x * 2 + y * fmt.w * 2];
        return v > value;
      });
}

// Thresholds an image in YUYV format, selecting pixels with a Y (luma) greater
// than value. The width must be a multiple of 4.
//
// This is implemented via some tricky bit shuffling that goes fast.
RangeImage FastYuyvYThreshold(ImageFormat fmt, const char *data, uint8_t value);

// Manages a pool of threads which do sharded thresholding.
class FastYuyvYPooledThresholder {
 public:
  // The number of threads we'll use.
  static constexpr int kThreads = 4;

  FastYuyvYPooledThresholder();
  // Shuts down and joins the threads.
  ~FastYuyvYPooledThresholder();

  // Actually does a threshold, merges the result, and returns it.
  RangeImage Threshold(ImageFormat fmt, const char *data, uint8_t value);

 private:
  enum class ThreadState {
    // Each thread moves itself into this state once it's done processing the
    // previous input data.
    kWaitingForInputData,
    // The main thread moves all the threads into this state once it has
    // finished setting up new input data.
    kProcessing,
  };

  // The main function for a thread.
  void RunThread(int index);

  // Returns true if all threads are currently done.
  bool AllThreadsDone() const {
    for (ThreadState state : states_) {
      if (state != ThreadState::kWaitingForInputData) {
        return false;
      }
    }
    return true;
  }

  std::array<std::thread, kThreads> threads_;
  // Protects access to the states_ and coordinates with condition_variable_.
  std::mutex mutex_;
  // Signals changes to states_ and quit_.
  std::condition_variable condition_variable_;
  bool quit_ = false;

  std::array<ThreadState, kThreads> states_;

  // Access to these is protected by coordination via states_.
  ImageFormat input_format_;
  const char *input_data_;
  uint8_t input_value_;
  std::array<RangeImage, kThreads> outputs_;
};

}  // namespace vision
}  // namespace aos

#endif  //  AOS_VISION_BLOB_THRESHOLD_H_
