#ifndef _Y2019_VISION_TARGET_FINDER_H_
#define _Y2019_VISION_TARGET_FINDER_H_

#include "aos/vision/blob/region_alloc.h"
#include "aos/vision/blob/threshold.h"
#include "aos/vision/blob/transpose.h"
#include "aos/vision/blob/contour.h"
#include "aos/vision/debug/overlay.h"
#include "aos/vision/math/vector.h"
#include "y2019/vision/target_types.h"

namespace y2019 {
namespace vision {

using aos::vision::ImageRange;
using aos::vision::RangeImage;
using aos::vision::BlobList;
using aos::vision::Vector;
using aos::vision::ContourNode;

class TargetFinder {
 public:
  TargetFinder();
  // Turn a raw image into blob range image.
  aos::vision::RangeImage Threshold(aos::vision::ImagePtr image);

  // Value against which we threshold.
  static uint8_t GetThresholdValue() { return 100; }

  // filter out obvious or durranged blobs.
  void PreFilter(BlobList *imgs);

  ContourNode *GetContour(const RangeImage &blob);
  ::std::vector<::Eigen::Vector2f> UnWarpContour(ContourNode *start) const;

  // Turn a blob into a polgygon.
  std::vector<aos::vision::Segment<2>> FillPolygon(
      const ::std::vector<::Eigen::Vector2f> &contour, bool verbose);

  // Turn a bloblist into components of a target.
  std::vector<TargetComponent> FillTargetComponentList(
      const std::vector<std::vector<aos::vision::Segment<2>>> &seg_list,
      bool verbose);

  // Piece the compenents together into a target.
  std::vector<Target> FindTargetsFromComponents(
      const std::vector<TargetComponent> component_list, bool verbose);

  // Given a target solve for the transformation of the template target.
  IntermediateResult ProcessTargetToResult(const Target &target, bool verbose);

  std::vector<IntermediateResult> FilterResults(
      const std::vector<IntermediateResult> &results, uint64_t print_rate);

  // Get the local overlay for debug if we are doing that.
  aos::vision::PixelLinesOverlay *GetOverlay() { return &overlay_; }

  // Convert target location into meters and radians.
  void GetAngleDist(const aos::vision::Vector<2> &target, double down_angle,
                    double *dist, double *angle);

  // Return the template target in a normalized space.
  const Target &GetTemplateTarget() { return target_template_; }

  const IntrinsicParams &intrinsics() const { return intrinsics_; }
  IntrinsicParams *mutable_intrinsics() { return &intrinsics_; }

 private:
  // Find a loosly connected target.
  double DetectConnectedTarget(const RangeImage &img);

  aos::vision::PixelLinesOverlay overlay_;

  aos::vision::AnalysisAllocator alloc_;

  // The template for the default target in the standard space.
  const Target target_template_;

  IntrinsicParams intrinsics_;

  ExtrinsicParams default_extrinsics_;

  // Counts for logging.
  size_t frame_count_;
  size_t valid_result_count_;
};

}  // namespace vision
}  // namespace y2019

#endif
