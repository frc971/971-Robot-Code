#ifndef _AOS_VIISON_BLOB_CONTOUR_H_
#define _AOS_VIISON_BLOB_CONTOUR_H_

#include "aos/vision/blob/range_image.h"
#include "aos/vision/blob/region_alloc.h"

namespace aos {
namespace vision {

// Countour nodes are slingly linked list chains of pixels that go around
// the boundary of a blob.
struct ContourNode {
  ContourNode(int x, int y) : pt({x, y}) { next = this; }
  ContourNode(int x, int y, ContourNode *next) : pt({x, y}), next(next) {}
  ContourNode() {}

  // Construction routine to attach a node to the end.
  // Prefer to manipulate contours using RangeImgToContour.
  ContourNode *append(int x, int y, AnalysisAllocator *alloc) {
    next = alloc->cons_obj<ContourNode>(x, y);
    return next;
  }
  // Construction routine to attach a node to the beginning.
  // Prefer to manipulate contours using RangeImgToContour.
  ContourNode *pappend(int x, int y, AnalysisAllocator *alloc) {
    return alloc->cons_obj<ContourNode>(x, y, this);
  }

  Point pt;
  ContourNode *next;
};

// Converts range image to contour using sweepline analysis.
ContourNode *RangeImgToContour(const RangeImage &rimg,
                               AnalysisAllocator *alloc);

}  // namespace vision
}  // namespace aos

#endif  //  _AOS_VIISON_BLOB_CONTOUR_H_
