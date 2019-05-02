#ifndef _Y2019_VISION_TARGET_FINDER_H_
#define _Y2019_VISION_TARGET_FINDER_H_

#include <memory>

#include "aos/vision/blob/contour.h"
#include "aos/vision/blob/region_alloc.h"
#include "aos/vision/blob/threshold.h"
#include "aos/vision/blob/transpose.h"
#include "aos/vision/debug/overlay.h"
#include "aos/vision/math/vector.h"
#include "y2019/vision/target_types.h"

namespace ceres {

class Context;

}  // namespace ceres
namespace y2019 {
namespace vision {

using aos::vision::ImageRange;
using aos::vision::RangeImage;
using aos::vision::BlobList;
using aos::vision::Vector;
using aos::vision::ContourNode;

struct Polygon {
  ::std::vector<aos::vision::Segment<2>> segments;
  ::std::vector<::Eigen::Vector2f> contour;
};

class TargetFinder {
 public:
  TargetFinder();
  ~TargetFinder();

  // Turn a raw image into blob range image.
  aos::vision::RangeImage Threshold(aos::vision::ImagePtr image);

  // Value against which we threshold.
  static uint8_t GetThresholdValue() { return 100; }

  int PixelCount(BlobList *imgs);

  // filter out obvious or durranged blobs.
  void PreFilter(BlobList *imgs);

  ContourNode *GetContour(const RangeImage &blob);
  ::std::vector<::Eigen::Vector2f> UnWarpContour(ContourNode *start) const;

  // Turn a blob into a polgygon.
  Polygon FindPolygon(::std::vector<::Eigen::Vector2f> &&contour, bool verbose);

  // Turn a bloblist into components of a target.
  std::vector<TargetComponent> FillTargetComponentList(
      const ::std::vector<Polygon> &seg_list, bool verbose);

  // Piece the compenents together into a target.
  std::vector<Target> FindTargetsFromComponents(
      const std::vector<TargetComponent> component_list, bool verbose);

  // Given a target solve for the transformation of the template target.
  //
  // This is safe to call concurrently.
  IntermediateResult ProcessTargetToResult(const Target &target, bool verbose);

  // Returns true if a target is good, and false otherwise.  Picks the 4 vs 8
  // point solution depending on which one looks right.
  bool MaybePickAndUpdateResult(IntermediateResult *result, bool verbose);

  std::vector<IntermediateResult> FilterResults(
      const std::vector<IntermediateResult> &results, uint64_t print_rate,
      bool verbose);

  bool TestExposure(const std::vector<IntermediateResult> &results,
                    int pixel_count, int *desired_exposure);

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

  const std::unique_ptr<ceres::Context> ceres_context_;

  // Counts for logging.
  size_t frame_count_;
  size_t valid_result_count_;

  int close_bucket_ = 0;

  int current_exposure_ = 0;
};

}  // namespace vision
}  // namespace y2019

#endif
