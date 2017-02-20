#include "aos/vision/blob/transpose.h"

#include <algorithm>

namespace aos {
namespace vision {

RangeImage Transpose(const RangeImage &img) {
  enum EventT {
    // Must happen before point adds and deletes.
    kRangeStart = 0,
    kRangeEnd = 1,
    // Non-overlapping
    kPointAdd = 3,
    kPointDel = 2,
  };
  std::vector<std::vector<std::pair<int, EventT>>> events;
  int y = img.min_y();
  for (const std::vector<ImageRange> &row : img) {
    for (const ImageRange &range : row) {
      if (range.ed >= static_cast<int>(events.size()))
        events.resize(range.ed + 1);
      events[range.st].emplace_back(y, kPointAdd);
      events[range.ed].emplace_back(y, kPointDel);
    }
    ++y;
  }

  int min_y = 0;
  while (min_y < (int)events.size() && events[min_y].empty()) ++min_y;

  std::vector<ImageRange> prev_ranges;
  std::vector<ImageRange> cur_ranges;

  std::vector<std::vector<ImageRange>> rows;
  for (int y = min_y; y < static_cast<int>(events.size()) - 1; ++y) {
    auto row_events = std::move(events[y]);
    for (const auto &range : prev_ranges) {
      row_events.emplace_back(range.st, kRangeStart);
      row_events.emplace_back(range.ed, kRangeEnd);
    }
    std::sort(row_events.begin(), row_events.end());
    cur_ranges.clear();

    bool has_cur_range = false;
    ImageRange cur_range{0, 0};
    auto add_range = [&](ImageRange range) {
      if (range.st == range.ed) return;
      if (has_cur_range) {
        if (cur_range.ed == range.st) {
          range = ImageRange{cur_range.st, range.ed};
        } else {
          cur_ranges.emplace_back(cur_range);
        }
      }
      cur_range = range;
      has_cur_range = true;
    };

    int prev_start;
    for (const auto &pt : row_events) {
      switch (pt.second) {
        case kRangeStart:
          prev_start = pt.first;
          break;
        case kPointAdd:
          add_range(ImageRange{pt.first, pt.first + 1});
          break;
        case kRangeEnd:
          add_range(ImageRange{prev_start, pt.first});
          break;
        case kPointDel:
          add_range(ImageRange{prev_start, pt.first});
          prev_start = pt.first + 1;
          break;
      }
    }

    if (has_cur_range) cur_ranges.emplace_back(cur_range);
    rows.emplace_back(cur_ranges);
    std::swap(cur_ranges, prev_ranges);
  }
  return RangeImage(min_y, std::move(rows));
}

}  // namespace vision
}  // namespace aos
