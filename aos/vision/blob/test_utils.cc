#include "aos/vision/blob/test_utils.h"

namespace aos {
namespace vision {

RangeImage LoadFromTestData(int mini, const char *data) {
  // Consume initial return.
  if (*data) ++data;
  std::vector<std::vector<ImageRange>> rows;
  int x = 0;
  bool p_score = false;
  int pstart = -1;
  std::vector<ImageRange> out_ranges;

  for (; *data; ++data) {
    char cell = *data;

    if (cell == '\n') {
      if (p_score) {
        out_ranges.emplace_back(ImageRange(pstart, x));
      }
      rows.emplace_back(out_ranges);
      out_ranges = {};
      x = 0;
      pstart = -1;
      p_score = false;
    } else {
      if ((cell != ' ') != p_score) {
        if (p_score) {
          out_ranges.emplace_back(ImageRange(pstart, x));
        } else {
          pstart = x;
        }
        p_score = !p_score;
      }
      ++x;
    }
  }
  return RangeImage(mini, std::move(rows));
}

}  // namespace vision
}  // namespace aos
