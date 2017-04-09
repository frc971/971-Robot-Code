#include <Eigen/Dense>
#include <iostream>

#include "y2017/vision/target_finder.h"

#include "aos/vision/blob/move_scale.h"
#include "aos/vision/blob/stream_view.h"
#include "aos/vision/blob/transpose.h"
#include "aos/vision/debug/debug_framework.h"
#include "aos/vision/math/vector.h"

using aos::vision::ImageRange;
using aos::vision::ImageFormat;
using aos::vision::RangeImage;
using aos::vision::BlobList;

namespace y2017 {
namespace vision {

BlobList RenderTargetListShifted(const std::vector<TargetComponent> &list) {
  BlobList out;
  for (const auto &entity : list) {
    out.emplace_back(entity.RenderShifted());
  }
  return out;
}

RangeImage TargetComponent::RenderShifted() const {
  std::vector<std::vector<ImageRange>> out_range_list;
  int y = 0;
  double max_y = -b / (2 * a);
  double parab_off = max_y * max_y * a + max_y * b;
  RangeImage t_img = Transpose(*img);
  for (const auto &row : t_img) {
    int off = -(y * y * a + y * b - parab_off);
    // int off = 0;
    // fprintf(stderr, "off: %d %d\n", off, y);
    std::vector<ImageRange> row_out;
    for (const ImageRange &range : row) {
      row_out.emplace_back(ImageRange{off + range.st, off + range.ed});
    }
    ++y;
    out_range_list.emplace_back(std::move(row_out));
  }
  return RangeImage(t_img.min_y(), std::move(out_range_list));
}

class FilterHarnessExample : public aos::vision::FilterHarness {
 public:
  aos::vision::RangeImage Threshold(aos::vision::ImagePtr image) override {
    return finder_.Threshold(image);
  }

  void InstallViewer(aos::vision::BlobStreamViewer *viewer) override {
    viewer_ = viewer;
    viewer_->SetScale(0.5);
    overlays_.push_back(&overlay_);
    overlays_.push_back(finder_.GetOverlay());
    viewer_->view()->SetOverlays(&overlays_);
  }

  bool HandleBlobs(BlobList imgs, ImageFormat /*fmt*/) override {
    // reset for next drawing cycle
    for (auto &overlay : overlays_) {
      overlay->Reset();
    }

    // Remove bad blobs.
    finder_.PreFilter(imgs);

    // calculate each component/
    std::vector<TargetComponent> target_component_list =
        finder_.FillTargetComponentList(imgs);

    DrawComponents(target_component_list);

    // Put the compenents together into targets and pick the best.
    Target final_target;
    bool found_target =
        finder_.FindTargetFromComponents(target_component_list, &final_target);

    // BlobList newImg = RenderTargetListShifted(target_component_list);
    if (viewer_) {
      viewer_->DrawBlobList(imgs, {0, 0, 255});
    }

    if (found_target) {
      BlobList list;
      list.emplace_back(*(final_target.comp1.img));
      list.emplace_back(*(final_target.comp2.img));
      viewer_->DrawBlobList(list, {0, 255, 0});
      overlay_.DrawCross(final_target.screen_coord, 25, {255, 255, 255});
    }

    // No targets.
    return found_target;
  }

  void DrawComponents(const std::vector<TargetComponent> comp) {
    for (const TargetComponent &t : comp) {
      aos::vision::ImageBBox bbox;
      GetBBox(*(t.img), &bbox);
      overlay_.DrawBBox(bbox, {255, 0, 0});

      overlay_.StartNewProfile();
      for (int i = 0; i < bbox.maxx - bbox.minx; i += 10) {
        double y0 = t.a * i * i + t.b * i + t.c_0;
        double y1 = t.a * i * i + t.b * i + t.c_1;
        overlay_.AddPoint(aos::vision::Vector<2>(i + t.mini, y0), {255, 0, 0});
        overlay_.AddPoint(aos::vision::Vector<2>(i + t.mini, y1), {255, 0, 0});
      }
    }
  }

 private:
  // implementation of the filter pipeline.
  TargetFinder finder_;
  aos::vision::BlobStreamViewer *viewer_ = nullptr;
  aos::vision::PixelLinesOverlay overlay_;
  std::vector<aos::vision::OverlayBase *> overlays_;
};

}  // namespace vision
}  // namespace y2017

int main(int argc, char **argv) {
  y2017::vision::FilterHarnessExample filter_harness;
  aos::vision::DebugFrameworkMain(argc, argv, &filter_harness,
                                  aos::vision::CameraParams());
}
