#include <Eigen/Dense>
#include <iostream>

#include "y2019/vision/target_finder.h"

#include "aos/vision/blob/move_scale.h"
#include "aos/vision/blob/stream_view.h"
#include "aos/vision/blob/transpose.h"
#include "aos/vision/debug/debug_framework.h"
#include "aos/vision/math/vector.h"

using aos::vision::ImageRange;
using aos::vision::ImageFormat;
using aos::vision::RangeImage;
using aos::vision::AnalysisAllocator;
using aos::vision::BlobList;
using aos::vision::Vector;
using aos::vision::Segment;
using aos::vision::PixelRef;

namespace y2019 {
namespace vision {

std::vector<PixelRef> GetNColors(size_t num_colors) {
  std::vector<PixelRef> colors;
  for (size_t i = 0; i < num_colors; ++i) {
    int quadrent = i * 6 / num_colors;
    uint8_t alpha = (256 * 6 * i - quadrent * num_colors * 256) / num_colors;
    uint8_t inv_alpha = 255 - alpha;
    switch (quadrent) {
      case 0:
        colors.push_back(PixelRef{255, alpha, 0});
        break;
      case 1:
        colors.push_back(PixelRef{inv_alpha, 255, 0});
        break;
      case 2:
        colors.push_back(PixelRef{0, 255, alpha});
        break;
      case 3:
        colors.push_back(PixelRef{0, inv_alpha, 255});
        break;
      case 4:
        colors.push_back(PixelRef{alpha, 0, 255});
        break;
      case 5:
        colors.push_back(PixelRef{255, 0, inv_alpha});
        break;
    }
  }
  return colors;
}

class FilterHarness : public aos::vision::FilterHarness {
 public:
  aos::vision::RangeImage Threshold(aos::vision::ImagePtr image) override {
    return finder_.Threshold(image);
  }

  void InstallViewer(aos::vision::BlobStreamViewer *viewer) override {
    viewer_ = viewer;
    viewer_->SetScale(0.75);
    overlays_.push_back(&overlay_);
    overlays_.push_back(finder_.GetOverlay());
    viewer_->view()->SetOverlays(&overlays_);
  }

  void DrawBlob(const RangeImage &blob, PixelRef color) {
    if (viewer_) {
      BlobList list;
      list.push_back(blob);
      viewer_->DrawBlobList(list, color);
    }
  }

  bool HandleBlobs(BlobList imgs, ImageFormat fmt) override {
    imgs_last_ = imgs;
    fmt_last_ = fmt;
    // reset for next drawing cycle
    for (auto &overlay : overlays_) {
      overlay->Reset();
    }

    if (draw_select_blob_ || draw_raw_poly_ || draw_components_ ||
        draw_raw_target_ || draw_raw_IR_ || draw_results_) {
      printf("_____ New Image _____\n");
    }

    // Remove bad blobs.
    finder_.PreFilter(&imgs);

    // Find polygons from blobs.
    std::vector<std::vector<Segment<2>>> raw_polys;
    for (const RangeImage &blob : imgs) {
      // Convert blobs to contours in the corrected space.
      ContourNode* contour = finder_.GetContour(blob);
      if (draw_contours_) {
        DrawContour(contour, {255, 0, 0});
      }
      finder_.UnWarpContour(contour);
      if (draw_contours_) {
        DrawContour(contour, {0, 0, 255});
      }

      // Process to polygons.
      std::vector<Segment<2>> polygon =
          finder_.FillPolygon(contour, draw_raw_poly_);
      if (polygon.empty()) {
        if (!draw_contours_) {
          DrawBlob(blob, {255, 0, 0});
        }
      } else {
        raw_polys.push_back(polygon);
        if (draw_select_blob_) {
          DrawBlob(blob, {0, 0, 255});
        }
        if (draw_raw_poly_) {
          std::vector<PixelRef> colors = GetNColors(polygon.size());
          std::vector<Vector<2>> corners;
          for (size_t i = 0; i < 4; ++i) {
            corners.push_back(polygon[i].Intersect(polygon[(i + 1) % 4]));
          }

          for (size_t i = 0; i < 4; ++i) {
            overlay_.AddLine(corners[i], corners[(i + 1) % 4], colors[i]);
          }
        }
      }
    }

    // Calculate each component side of a possible target.
    std::vector<TargetComponent> target_component_list =
        finder_.FillTargetComponentList(raw_polys);
    if (draw_components_) {
      for (const TargetComponent &comp : target_component_list) {
        DrawComponent(comp, {0, 255, 255}, {0, 255, 255}, {255, 0, 0},
                      {0, 0, 255});
      }
    }

    // Put the compenents together into targets.
    std::vector<Target> target_list = finder_.FindTargetsFromComponents(
        target_component_list, draw_raw_target_);
    if (draw_raw_target_) {
      for (const Target &target : target_list) {
        DrawTarget(target);
      }
    }

    // Use the solver to generate an intermediate version of our results.
    std::vector<IntermediateResult> results;
    for (const Target &target : target_list) {
      results.emplace_back(finder_.ProcessTargetToResult(target, draw_raw_IR_));
      if (draw_raw_IR_) DrawResult(results.back(), {255, 128, 0});
    }

    // Check that our current results match possible solutions.
    results = finder_.FilterResults(results, 0);
    if (draw_results_) {
      for (const IntermediateResult &res : results) {
        DrawTarget(res, {0, 255, 0});
      }
    }

    // If the target list is not empty then we found a target.
    return !results.empty();
  }

  std::function<void(uint32_t)> RegisterKeyPress() override {
    return [this](uint32_t key) {
      (void)key;
      if (key == 'z') {
        draw_results_ = !draw_results_;
      } else if (key == 'x') {
        draw_raw_IR_ = !draw_raw_IR_;
      } else if (key == 'c') {
        draw_raw_target_ = !draw_raw_target_;
      } else if (key == 'v') {
        draw_components_ = !draw_components_;
      } else if (key == 'b') {
        draw_raw_poly_ = !draw_raw_poly_;
      } else if (key == 'n') {
        draw_contours_ = !draw_contours_;
      } else if (key == 'm') {
        draw_select_blob_ = !draw_select_blob_;
      } else if (key == 'h') {
        printf("Key Mappings:\n");
        printf(" z: Toggle drawing final target pose.\n");
        printf(" x: Toggle drawing re-projected targets and print solver results.\n");
        printf(" c: Toggle drawing proposed target groupings.\n");
        printf(" v: Toggle drawing ordered target components.\n");
        printf(" b: Toggle drawing proposed target components.\n");
        printf(" n: Toggle drawing countours before and after warping.\n");
        printf(" m: Toggle drawing raw blob data (may need to change image to toggle a redraw).\n");
        printf(" h: Print this message.\n");
        printf(" a: May log camera image to /tmp/debug_viewer_jpeg_<#>.yuyv\n");
        printf(" q: Exit the application.\n");
      } else if (key == 'q') {
        printf("User requested shutdown.\n");
        exit(0);
      }
      HandleBlobs(imgs_last_, fmt_last_);
      viewer_->Redraw();
    };
  }

  void DrawContour(ContourNode *contour, PixelRef color) {
    if (viewer_) {
      for (ContourNode *node = contour; node->next != contour;) {
        Vector<2> a(node->pt.x, node->pt.y);
        Vector<2> b(node->next->pt.x, node->next->pt.y);
        overlay_.AddLine(a, b, color);
        node = node->next;
      }
    }
  }

  void DrawComponent(const TargetComponent &comp, PixelRef top_color,
                     PixelRef bot_color, PixelRef in_color,
                     PixelRef out_color) {
    overlay_.AddLine(comp.top, comp.inside, top_color);
    overlay_.AddLine(comp.bottom, comp.outside, bot_color);

    overlay_.AddLine(comp.bottom, comp.inside, in_color);
    overlay_.AddLine(comp.top, comp.outside, out_color);
  }

  void DrawTarget(const Target &target) {
    Vector<2> leftTop = (target.left.top + target.left.inside) * 0.5;
    Vector<2> rightTop = (target.right.top + target.right.inside) * 0.5;
    overlay_.AddLine(leftTop, rightTop, {255, 215, 0});

    Vector<2> leftBot = (target.left.bottom + target.left.outside) * 0.5;
    Vector<2> rightBot = (target.right.bottom + target.right.outside) * 0.5;
    overlay_.AddLine(leftBot, rightBot, {255, 215, 0});

    overlay_.AddLine(leftTop, leftBot, {255, 215, 0});
    overlay_.AddLine(rightTop, rightBot, {255, 215, 0});
  }

  void DrawResult(const IntermediateResult &result, PixelRef color) {
    Target target =
        Project(finder_.GetTemplateTarget(), intrinsics(), result.extrinsics);
    DrawComponent(target.left, color, color, color, color);
    DrawComponent(target.right, color, color, color, color);
  }

  void DrawTarget(const IntermediateResult &result, PixelRef color) {
    Target target =
        Project(finder_.GetTemplateTarget(), intrinsics(), result.extrinsics);
    Segment<2> leftAx((target.left.top + target.left.inside) * 0.5,
                      (target.left.bottom + target.left.outside) * 0.5);
    leftAx.Set(leftAx.A() * 0.9 + leftAx.B() * 0.1,
               leftAx.B() * 0.9 + leftAx.A() * 0.1);
    overlay_.AddLine(leftAx, color);

    Segment<2> rightAx((target.right.top + target.right.inside) * 0.5,
                       (target.right.bottom + target.right.outside) * 0.5);
    rightAx.Set(rightAx.A() * 0.9 + rightAx.B() * 0.1,
                rightAx.B() * 0.9 + rightAx.A() * 0.1);
    overlay_.AddLine(rightAx, color);

    overlay_.AddLine(leftAx.A(), rightAx.A(), color);
    overlay_.AddLine(leftAx.B(), rightAx.B(), color);
    Vector<3> p1(0.0, 0.0, 100.0);

    Vector<3> p2 =
        Rotate(intrinsics().mount_angle, result.extrinsics.r1, 0.0, p1);
    Vector<2> p3(p2.x(), p2.y());
    overlay_.AddLine(leftAx.A(), p3 + leftAx.A(), {0, 255, 0});
    overlay_.AddLine(leftAx.B(), p3 + leftAx.B(), {0, 255, 0});
    overlay_.AddLine(rightAx.A(), p3 + rightAx.A(), {0, 255, 0});
    overlay_.AddLine(rightAx.B(), p3 + rightAx.B(), {0, 255, 0});

    overlay_.AddLine(p3 + leftAx.A(), p3 + leftAx.B(), {0, 255, 0});
    overlay_.AddLine(p3 + leftAx.A(), p3 + rightAx.A(), {0, 255, 0});
    overlay_.AddLine(p3 + rightAx.A(), p3 + rightAx.B(), {0, 255, 0});
    overlay_.AddLine(p3 + leftAx.B(), p3 + rightAx.B(), {0, 255, 0});
  }

  const IntrinsicParams &intrinsics() const { return finder_.intrinsics(); }

 private:
  // implementation of the filter pipeline.
  TargetFinder finder_;
  aos::vision::BlobStreamViewer *viewer_ = nullptr;
  aos::vision::PixelLinesOverlay overlay_;
  std::vector<aos::vision::OverlayBase *> overlays_;
  BlobList imgs_last_;
  ImageFormat fmt_last_;
  bool draw_select_blob_ = false;
  bool draw_contours_ = false;
  bool draw_raw_poly_ = false;
  bool draw_components_ = false;
  bool draw_raw_target_ = false;
  bool draw_raw_IR_ = false;
  bool draw_results_ = true;
};

}  // namespace vision
}  // namespace y2017

int main(int argc, char **argv) {
  y2019::vision::FilterHarness filter_harness;
  aos::vision::DebugFrameworkMain(argc, argv, &filter_harness,
                                  aos::vision::CameraParams());
}
