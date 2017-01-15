#ifndef _AOS_VISION_BLOB_HIERARCHICAL_CONTOUR_MERGE_H_
#define _AOS_VISION_BLOB_HIERARCHICAL_CONTOUR_MERGE_H_

#include <vector>

#include "aos/vision/blob/contour.h"
#include "aos/vision/blob/range_image.h"

namespace aos {
namespace vision {

struct FittedLine {
  Point st;
  Point ed;
};

// Merges a contour into a list of best fit lines where the regression value is
// merge_rate and only emit lines at least min_len pixels long.
void HierarchicalMerge(ContourNode *stval, std::vector<FittedLine> *fit_lines,
                       float merge_rate = 4.0, int min_len = 15);

}  // namespace vision
}  // namespace aos

#endif  // _AOS_VISION_BLOB_HIERARCHICAL_CONTOUR_MERGE_H_
