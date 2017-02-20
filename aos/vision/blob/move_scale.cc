#include "aos/vision/blob/move_scale.h"

namespace aos {
namespace vision {

RangeImage MoveScale(const RangeImage &img, int dx, int dy, int scale) {
  std::vector<std::vector<ImageRange>> out_range_list;
  for (const auto &range_list : img) {
    std::vector<ImageRange> ranges;
    for (const auto &range : range_list) {
      ranges.emplace_back((range.st + dx) * scale, (range.ed + dx) * scale);
    }
    for (int i = 1; i < scale; ++i) {
      out_range_list.push_back(ranges);
    }
    out_range_list.emplace_back(std::move(ranges));
  }
  return RangeImage((img.mini() + dy) * scale, std::move(out_range_list));
}

std::vector<RangeImage> MoveScale(const std::vector<RangeImage> &imgs, int dx,
                                  int dy, int scale) {
  std::vector<RangeImage> out;
  for (const auto &img : imgs) {
    out.emplace_back(MoveScale(img, dx, dy, scale));
  }
  return out;
}

void GetBBox(const RangeImage &img, ImageBBox *bbox) {
  bbox->miny = std::min(img.min_y(), bbox->miny);
  bbox->maxy = std::max(img.min_y() + img.size(), bbox->maxy);
  for (const auto &range_lst : img) {
    if (range_lst.size() > 0) {
      bbox->maxx = std::max(range_lst[range_lst.size() - 1].ed, bbox->maxx);
      bbox->minx = std::min(range_lst[0].st, bbox->minx);
    }
  }
}

}  // namespace vision
}  // namespace aos
