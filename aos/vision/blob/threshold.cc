#include "aos/vision/blob/threshold.h"

namespace aos {
namespace vision {

#define MASH(v0, v1, v2, v3, v4)                                  \
  ((uint8_t(v0) << 4) | (uint8_t(v1) << 3) | (uint8_t(v2) << 2) | \
   (uint8_t(v3) << 1) | (uint8_t(v4)))

RangeImage FastYuyvYThreshold(ImageFormat fmt, const char *data,
                              uint8_t value) {
  std::vector<std::vector<ImageRange>> ranges;
  ranges.reserve(fmt.h);
  for (int y = 0; y < fmt.h; ++y) {
    const char *row = fmt.w * y * 2 + data;
    bool p_score = false;
    int pstart = -1;
    std::vector<ImageRange> rngs;
    for (int x = 0; x < fmt.w / 4; ++x) {
      uint8_t v[8];
      memcpy(&v[0], row + x * 4 * 2, 8);
      uint8_t pattern =
          MASH(p_score, v[0] > value, v[2] > value, v[4] > value, v[6] > value);
      switch (pattern) {
        /*
# Ruby code to generate the below code:
32.times do |v|
        puts "case MASH(#{[v[4], v[3], v[2], v[1], v[0]].join(", ")}):"
        p_score = v[4]
        pstart = "pstart"
        4.times do |i|
                if v[3 - i] != p_score
                        if (p_score == 1)
                                puts "  rngs.emplace_back(ImageRange(#{pstart},
x * 4 + #{i}));"
                        else
                                pstart = "x * 4 + #{i}"
                        end
                        p_score = v[3 - i]
                end
        end
        if (pstart != "pstart")
                puts "  pstart = #{pstart};"
        end
        if (p_score != v[4])
                puts "  p_score = #{["false", "true"][v[0]]};"
        end
        puts "  break;"
end
*/
        case MASH(0, 0, 0, 0, 0):
          break;
        case MASH(0, 0, 0, 0, 1):
          pstart = x * 4 + 3;
          p_score = true;
          break;
        case MASH(0, 0, 0, 1, 0):
          rngs.emplace_back(ImageRange(x * 4 + 2, x * 4 + 3));
          pstart = x * 4 + 2;
          break;
        case MASH(0, 0, 0, 1, 1):
          pstart = x * 4 + 2;
          p_score = true;
          break;
        case MASH(0, 0, 1, 0, 0):
          rngs.emplace_back(ImageRange(x * 4 + 1, x * 4 + 2));
          pstart = x * 4 + 1;
          break;
        case MASH(0, 0, 1, 0, 1):
          rngs.emplace_back(ImageRange(x * 4 + 1, x * 4 + 2));
          pstart = x * 4 + 3;
          p_score = true;
          break;
        case MASH(0, 0, 1, 1, 0):
          rngs.emplace_back(ImageRange(x * 4 + 1, x * 4 + 3));
          pstart = x * 4 + 1;
          break;
        case MASH(0, 0, 1, 1, 1):
          pstart = x * 4 + 1;
          p_score = true;
          break;
        case MASH(0, 1, 0, 0, 0):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 1));
          pstart = x * 4 + 0;
          break;
        case MASH(0, 1, 0, 0, 1):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 1));
          pstart = x * 4 + 3;
          p_score = true;
          break;
        case MASH(0, 1, 0, 1, 0):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 1));
          rngs.emplace_back(ImageRange(x * 4 + 2, x * 4 + 3));
          pstart = x * 4 + 2;
          break;
        case MASH(0, 1, 0, 1, 1):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 1));
          pstart = x * 4 + 2;
          p_score = true;
          break;
        case MASH(0, 1, 1, 0, 0):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 2));
          pstart = x * 4 + 0;
          break;
        case MASH(0, 1, 1, 0, 1):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 2));
          pstart = x * 4 + 3;
          p_score = true;
          break;
        case MASH(0, 1, 1, 1, 0):
          rngs.emplace_back(ImageRange(x * 4 + 0, x * 4 + 3));
          pstart = x * 4 + 0;
          break;
        case MASH(0, 1, 1, 1, 1):
          pstart = x * 4 + 0;
          p_score = true;
          break;
        case MASH(1, 0, 0, 0, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          p_score = false;
          break;
        case MASH(1, 0, 0, 0, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          pstart = x * 4 + 3;
          break;
        case MASH(1, 0, 0, 1, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          rngs.emplace_back(ImageRange(x * 4 + 2, x * 4 + 3));
          pstart = x * 4 + 2;
          p_score = false;
          break;
        case MASH(1, 0, 0, 1, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          pstart = x * 4 + 2;
          break;
        case MASH(1, 0, 1, 0, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          rngs.emplace_back(ImageRange(x * 4 + 1, x * 4 + 2));
          pstart = x * 4 + 1;
          p_score = false;
          break;
        case MASH(1, 0, 1, 0, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          rngs.emplace_back(ImageRange(x * 4 + 1, x * 4 + 2));
          pstart = x * 4 + 3;
          break;
        case MASH(1, 0, 1, 1, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          rngs.emplace_back(ImageRange(x * 4 + 1, x * 4 + 3));
          pstart = x * 4 + 1;
          p_score = false;
          break;
        case MASH(1, 0, 1, 1, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 0));
          pstart = x * 4 + 1;
          break;
        case MASH(1, 1, 0, 0, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 1));
          p_score = false;
          break;
        case MASH(1, 1, 0, 0, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 1));
          pstart = x * 4 + 3;
          break;
        case MASH(1, 1, 0, 1, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 1));
          rngs.emplace_back(ImageRange(x * 4 + 2, x * 4 + 3));
          pstart = x * 4 + 2;
          p_score = false;
          break;
        case MASH(1, 1, 0, 1, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 1));
          pstart = x * 4 + 2;
          break;
        case MASH(1, 1, 1, 0, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 2));
          p_score = false;
          break;
        case MASH(1, 1, 1, 0, 1):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 2));
          pstart = x * 4 + 3;
          break;
        case MASH(1, 1, 1, 1, 0):
          rngs.emplace_back(ImageRange(pstart, x * 4 + 3));
          p_score = false;
          break;
        case MASH(1, 1, 1, 1, 1):
          break;
      }

      for (int i = 0; i < 4; ++i) {
        if ((v[i * 2] > value) != p_score) {
          if (p_score) {
            rngs.emplace_back(ImageRange(pstart, x * 4 + i));
          } else {
            pstart = x * 4 + i;
          }
          p_score = !p_score;
        }
      }
    }
    if (p_score) {
      rngs.emplace_back(ImageRange(pstart, fmt.w));
    }
    ranges.push_back(rngs);
  }
  return RangeImage(0, std::move(ranges));
}

#undef MASH

}  // namespace vision
}  // namespace aos
