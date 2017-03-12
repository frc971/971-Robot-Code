#ifndef _Y2017_VISION_TARGET_FINDER_H_
#define _Y2017_VISION_TARGET_FINDER_H_

#include "aos/vision/blob/threshold.h"
#include "aos/vision/blob/transpose.h"
#include "aos/vision/debug/overlay.h"
#include "aos/vision/math/vector.h"

using aos::vision::ImageRange;
using aos::vision::RangeImage;
using aos::vision::BlobList;
using aos::vision::Vector;

namespace y2017 {
namespace vision {

// This polynomial exists in transpose space.
struct TargetComponent {
  const RangeImage *img = nullptr;

  // Polynomial constants.
  double a;
  double b;
  double c_0;
  double c_1;

  double mini;

  double CenterPolyOne() { return -b / (2.0 * a); }
  double CenterPolyTwo() { return (c_0); }

  double EvalMinAt(double c) {
    double min = CenterPolyOne();
    return a * (min * min) + b * min + c;
  }

  double EvalMinTop() { return EvalMinAt(c_1); }
  double EvalMinBot() { return EvalMinAt(c_0); }

  // Fit error is not normalized by n.
  double fit_error;
  int n;

  RangeImage RenderShifted() const;
};

// Convert back to screen space for final result.
struct Target {
  TargetComponent comp1;

  TargetComponent comp2;
  aos::vision::Vector<2> screen_coord;
};

class TargetFinder {
 public:
  // Turn a bloblist into components of a target.
  std::vector<TargetComponent> FillTargetComponentList(const BlobList &blobs);

  // Turn a raw image into blob range image.
  aos::vision::RangeImage Threshold(aos::vision::ImagePtr image);

  // filter out obvious or durranged blobs.
  void PreFilter(BlobList &imgs);

  // Piece the compenents together into a target.
  bool FindTargetFromComponents(std::vector<TargetComponent> component_list,
                                Target *final_target);

  // Get the local overlay for debug if we are doing that.
  aos::vision::PixelLinesOverlay *GetOverlay() { return &overlay_; }

  // Convert target location into meters and radians.
  void GetAngleDist(const aos::vision::Vector<2>& target, double down_angle,
                    double *dist, double *angle);

 private:
  // Find a loosly connected target.
  double DetectConnectedTarget(const RangeImage &img);

  // TODO(ben): move to overlay
  void DrawCross(aos::vision::Vector<2> center, aos::vision::PixelRef color);

  aos::vision::PixelLinesOverlay overlay_;
};

}  // namespace vision
}  // namespace y2017

#endif  // _Y2017_VISION_TARGET_FINDER_H_
