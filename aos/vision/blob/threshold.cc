#include "aos/vision/blob/threshold.h"

#include "aos/logging/logging.h"

namespace aos {
namespace vision {
namespace {

constexpr int kChunkSize = 8;

}  // namespace

// At a high level, the algorithm is the same as the slow thresholding, except
// it operates in kChunkSize-pixel chunks.
RangeImage FastYuyvYThreshold(ImageFormat fmt, const char *data,
                              uint8_t value) {
  AOS_CHECK_EQ(0, fmt.w % kChunkSize);
  std::vector<std::vector<ImageRange>> result;
  result.reserve(fmt.h);

  // Iterate through each row.
  for (int y = 0; y < fmt.h; ++y) {
    // The start of the data for the current row.
    const char *const current_row = fmt.w * y * 2 + data;
    bool in_range = false;
    int current_range_start = -1;
    std::vector<ImageRange> current_row_ranges;
    // Iterate through each kChunkSize-pixel chunk
    for (int x = 0; x < fmt.w / kChunkSize; ++x) {
      // The per-channel (YUYV) values in the current chunk.
      uint8_t chunk_channels[2 * kChunkSize];
      memcpy(&chunk_channels[0], current_row + x * kChunkSize * 2, 2 * kChunkSize);
      __builtin_prefetch(current_row + (x + 1) * kChunkSize * 2);

      for (int i = 0; i < kChunkSize; ++i) {
        if ((chunk_channels[i * 2] > value) != in_range) {
          const int here = x * kChunkSize + i;
          if (in_range) {
            current_row_ranges.emplace_back(ImageRange(current_range_start, here));
          } else {
            current_range_start = here;
          }
          in_range = !in_range;
        }
      }
    }
    if (in_range) {
      current_row_ranges.emplace_back(ImageRange(current_range_start, fmt.w));
    }
    result.push_back(current_row_ranges);
  }
  return RangeImage(0, std::move(result));
}

FastYuyvYPooledThresholder::FastYuyvYPooledThresholder() {
  states_.fill(ThreadState::kWaitingForInputData);
  for (int i = 0; i < kThreads; ++i) {
    threads_[i] = std::thread([this, i]() { RunThread(i); });
  }
}

FastYuyvYPooledThresholder::~FastYuyvYPooledThresholder() {
  {
    std::unique_lock<std::mutex> locker(mutex_);
    quit_ = true;
    condition_variable_.notify_all();
  }
  for (int i = 0; i < kThreads; ++i) {
    threads_[i].join();
  }
}

RangeImage FastYuyvYPooledThresholder::Threshold(ImageFormat fmt,
                                                 const char *data,
                                                 uint8_t value) {
  input_format_ = fmt;
  input_data_ = data;
  input_value_ = value;
  {
    std::unique_lock<std::mutex> locker(mutex_);
    for (int i = 0; i < kThreads; ++i) {
      states_[i] = ThreadState::kProcessing;
    }
    condition_variable_.notify_all();
    while (!AllThreadsDone()) {
      condition_variable_.wait(locker);
    }
  }
  std::vector<std::vector<ImageRange>> result;
  result.reserve(fmt.h);
  for (int i = 0; i < kThreads; ++i) {
    result.insert(result.end(), outputs_[i].begin(), outputs_[i].end());
  }
  return RangeImage(0, std::move(result));
}

void FastYuyvYPooledThresholder::RunThread(int i) {
  while (true) {
    {
      std::unique_lock<std::mutex> locker(mutex_);
      while (states_[i] == ThreadState::kWaitingForInputData) {
        if (quit_) {
          return;
        }
        condition_variable_.wait(locker);
      }
    }

    ImageFormat shard_format = input_format_;
    AOS_CHECK_EQ(shard_format.h % kThreads, 0);
    shard_format.h /= kThreads;

    outputs_[i] = FastYuyvYThreshold(
        shard_format, input_data_ + shard_format.w * 2 * shard_format.h * i,
        input_value_);
    {
      std::unique_lock<std::mutex> locker(mutex_);
      states_[i] = ThreadState::kWaitingForInputData;
      condition_variable_.notify_all();
    }
  }
}

}  // namespace vision
}  // namespace aos
