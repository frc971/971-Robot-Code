#include "aos/vision/blob/range_image.h"

#include <math.h>
#include <algorithm>

namespace aos {
namespace vision {

void DrawRangeImage(const RangeImage &rimg, ImagePtr outbuf, PixelRef color) {
  for (int i = 0; i < (int)rimg.ranges().size(); ++i) {
    int y = rimg.min_y() + i;
    for (ImageRange rng : rimg.ranges()[i]) {
      for (int x = rng.st; x < rng.ed; ++x) {
        outbuf.get_px(x, y) = color;
      }
    }
  }
}

RangeImage MergeRangeImage(const BlobList &blobl) {
  if (blobl.size() == 1) return blobl[0];

  int min_y = blobl[0].min_y();
  for (const RangeImage &subrimg : blobl) {
    if (min_y > subrimg.min_y()) min_y = subrimg.min_y();
  }
  std::vector<std::vector<ImageRange>> ranges;
  int i = min_y;
  while (true) {
    std::vector<ImageRange> range_lst;
    int n_missing = 0;
    for (const RangeImage &subrimg : blobl) {
      if (subrimg.min_y() > i) continue;
      int ri = i - subrimg.min_y();
      if (ri < (int)subrimg.ranges().size()) {
        for (const auto &span : subrimg.ranges()[ri]) {
          range_lst.emplace_back(span);
        }
      } else {
        ++n_missing;
      }
    }
    std::sort(range_lst.begin(), range_lst.end());
    ranges.emplace_back(std::move(range_lst));
    if (n_missing == static_cast<int>(blobl.size()))
      return RangeImage(min_y, std::move(ranges));
    ++i;
  }
}

std::string ShortDebugPrint(const BlobList &blobl) {
  RangeImage rimg = MergeRangeImage(blobl);
  std::string out;
  out += "{";
  out += "min_y: " + std::to_string(rimg.min_y());
  for (const auto &line : rimg) {
    out += "{";
    for (const auto &span : line) {
      out +=
          "{" + std::to_string(span.st) + ", " + std::to_string(span.ed) + "},";
    }
    out += "},";
  }
  out += "}";
  return out;
}

void DebugPrint(const BlobList &blobl) {
  RangeImage rimg = MergeRangeImage(blobl);
  int minx = rimg.ranges()[0][0].st;
  int maxx = 0;
  for (const auto &range : rimg.ranges()) {
    for (const auto &span : range) {
      if (span.st < minx) minx = span.st;
      if (span.ed > maxx) maxx = span.ed;
    }
  }
  printf("maxx: %d minx: %d\n", maxx, minx);
  char buf[maxx - minx];
  for (const auto &range : rimg.ranges()) {
    int i = minx;
    for (const auto &span : range) {
      for (; i < span.st; ++i) buf[i - minx] = ' ';
      for (; i < span.ed; ++i) buf[i - minx] = '#';
    }
    for (; i < maxx; ++i) buf[i - minx] = ' ';
    printf("%.*s\n", maxx - minx, buf);
  }
}

void RangeImage::Flip(int image_width, int image_height) {
  std::reverse(ranges_.begin(), ranges_.end());
  for (std::vector<ImageRange> &range : ranges_) {
    std::reverse(range.begin(), range.end());
    for (ImageRange &interval : range) {
      int tmp = image_width - interval.ed;
      interval.ed = image_width - interval.st;
      interval.st = tmp;
    }
  }

  min_y_ = image_height - static_cast<int>(ranges_.size()) - min_y_;
}

int RangeImage::npixels() {
  if (npixelsc_ > 0) {
    return npixelsc_;
  }
  npixelsc_ = calc_area();
  return npixelsc_;
}

int RangeImage::calc_area() const {
  int area = 0;
  for (auto &range : ranges_) {
    for (auto &interval : range) {
      area += interval.calc_width();
    }
  }
  return area;
}

}  // namespace vision
}  // namespace aos
