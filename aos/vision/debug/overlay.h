#ifndef _AOS_VISION_IMAGE_DEBUG_OVERLAY_H_
#define _AOS_VISION_IMAGE_DEBUG_OVERLAY_H_

#include <string>
#include <vector>

#include "aos/vision/image/image_types.h"
#include "aos/vision/math/segment.h"
#include "aos/vision/math/vector.h"

namespace aos {
namespace vision {

// Abstract away rendering to avoid compiling gtk for arm.
// This should match a reduced cairo rendering api.
class RenderInterface {
 public:
  RenderInterface() {}
  RenderInterface(RenderInterface &&other) = delete;
  RenderInterface(const RenderInterface &other) = delete;
  ~RenderInterface() {}

  virtual void Translate(double x, double y) = 0;
  virtual void SetSourceRGB(double r, double g, double b) = 0;
  virtual void MoveTo(double x, double y) = 0;
  virtual void LineTo(double x, double y) = 0;
  virtual void Circle(double x, double y, double r) = 0;
  // negative in x, y, text_x, text_y measures from max in those value
  virtual void Text(int x, int y, int text_x, int text_y,
                    const std::string &text) = 0;
  virtual void Stroke() = 0;
};

// Interface for a list of overlays to be drawn onto a debug image.
// These will be passed into the running vision algorithms to output debug info,
// so they must not have costly side-effects.
class OverlayBase {
 public:
  OverlayBase() {}
  virtual ~OverlayBase() {}

  // Draws this overlay to the given canvas.
  virtual void Draw(RenderInterface *render, double /* width */,
                    double /* height */) = 0;

  // Clears the entire overlay.
  virtual void Reset() = 0;

  PixelRef color = {255, 0, 0};
  double scale = 1.0;
};

// A lambda that renders directly to the render interface.
class LambdaOverlay : public OverlayBase {
 public:
  std::function<void(RenderInterface *, double, double)> draw_fn;
  void Draw(RenderInterface *render, double width, double height) override {
    if (draw_fn) draw_fn(render, width, height);
  }
  void Reset() override {}
};

// Lines rendered in a coordinate system where the origin is the center
// of the screen
class LinesOverlay : public OverlayBase {
 public:
  LinesOverlay() : OverlayBase() {}
  ~LinesOverlay() {}

  // build a segment for this line
  void add_line(Vector<2> st, Vector<2> ed) { add_line(st, ed, color); }

  // build a segment for this line
  void add_line(Vector<2> st, Vector<2> ed, PixelRef newColor) {
    lines_.emplace_back(
        std::pair<Segment<2>, PixelRef>(Segment<2>(st, ed), newColor));
  }

  void add_point(Vector<2> pt) { add_point(pt, color); }

  // add a new point connected to the last point in the line
  void add_point(Vector<2> pt, PixelRef newColor) {
    if (lines_.empty()) {
      lines_.emplace_back(
          std::pair<Segment<2>, PixelRef>(Segment<2>(pt, pt), newColor));
    } else {
      Vector<2> st = lines_.back().first.B();
      lines_.emplace_back(
          std::pair<Segment<2>, PixelRef>(Segment<2>(st, pt), newColor));
    }
  }

  void Draw(RenderInterface *render, double w, double h) override {
    render->Translate(w / 2.0, h / 2.0);
    for (const auto &ln : lines_) {
      PixelRef localColor = ln.second;
      render->SetSourceRGB(localColor.r / 255.0, localColor.g / 255.0,
                           localColor.b / 255.0);
      render->MoveTo(scale * ln.first.A().x(), -scale * ln.first.A().y());
      render->LineTo(scale * ln.first.B().x(), -scale * ln.first.B().y());
      render->Stroke();
    }
  }

  // Empting the list will blank the whole overlay
  void Reset() override { lines_.clear(); }

 private:
  // lines in this over lay
  std::vector<std::pair<Segment<2>, PixelRef>> lines_;
};

// Lines rendered in pixel coordinates (Should match up with the screen.)
class PixelLinesOverlay : public OverlayBase {
 public:
  PixelLinesOverlay() : OverlayBase() {}
  ~PixelLinesOverlay() {}

  // build a segment for this line
  void AddLine(Vector<2> st, Vector<2> ed) { AddLine(st, ed, color); }

  // build a segment for this line
  void AddLine(Vector<2> st, Vector<2> ed, PixelRef newColor) {
    lines_.emplace_back(
        std::pair<Segment<2>, PixelRef>(Segment<2>(st, ed), newColor));
  }

  void DrawCross(aos::vision::Vector<2> center, int width,
                 aos::vision::PixelRef color) {
    using namespace aos::vision;
    AddLine(Vector<2>(center.x() - width / 2, center.y()),
            Vector<2>(center.x() + width / 2, center.y()), color);
    AddLine(Vector<2>(center.x(), center.y() - width / 2),
            Vector<2>(center.x(), center.y() + width / 2), color);
  }

  void DrawBBox(const ImageBBox &box, aos::vision::PixelRef color) {
    using namespace aos::vision;
    AddLine(Vector<2>(box.minx, box.miny), Vector<2>(box.maxx, box.miny),
            color);
    AddLine(Vector<2>(box.maxx, box.miny), Vector<2>(box.maxx, box.maxy),
            color);
    AddLine(Vector<2>(box.maxx, box.maxy), Vector<2>(box.minx, box.maxy),
            color);
    AddLine(Vector<2>(box.minx, box.maxy), Vector<2>(box.minx, box.miny),
            color);
  }

  // Build a circle as a point and radius.
  void DrawCircle(Vector<2> center, double radius, PixelRef newColor) {
    circles_.emplace_back(std::pair<Vector<2>, std::pair<double, PixelRef>>(
        center, std::pair<double, PixelRef>(radius, newColor)));
  }

  void StartNewProfile() { start_profile = true; }

  // add a new point connected to the last point in the line
  void AddPoint(Vector<2> pt, PixelRef newColor) {
    if (lines_.empty() || start_profile) {
      lines_.emplace_back(
          std::pair<Segment<2>, PixelRef>(Segment<2>(pt, pt), newColor));
      start_profile = false;
    } else {
      Vector<2> st = lines_.back().first.B();
      lines_.emplace_back(
          std::pair<Segment<2>, PixelRef>(Segment<2>(st, pt), newColor));
    }
  }

  void Draw(RenderInterface *render, double /*width*/,
            double /*hieght*/) override {
    for (const auto &ln : lines_) {
      PixelRef localColor = ln.second;
      render->SetSourceRGB(localColor.r / 255.0, localColor.g / 255.0,
                           localColor.b / 255.0);
      render->MoveTo(ln.first.A().x(), ln.first.A().y());
      render->LineTo(ln.first.B().x(), ln.first.B().y());
      render->Stroke();
    }
    for (const auto &circle : circles_) {
      PixelRef localColor = circle.second.second;
      render->SetSourceRGB(localColor.r / 255.0, localColor.g / 255.0,
                           localColor.b / 255.0);
      render->Circle(circle.first.x(), circle.first.y(), circle.second.first);
      render->Stroke();
    }
  }

  // Empting the list will blank the whole overlay.
  void Reset() override {
    lines_.clear();
    circles_.clear();
  }

 private:
  // Lines in this overlay.
  std::vector<std::pair<Segment<2>, PixelRef>> lines_;
  std::vector<std::pair<Vector<2>, std::pair<double, PixelRef>>> circles_;
  bool start_profile = false;
};

// Circles rendered in a coordinate system where the origin is the center
// of the screen.
class CircleOverlay : public OverlayBase {
 public:
  CircleOverlay() : OverlayBase() {}
  ~CircleOverlay() {}

  // build a circle as a point and radius
  std::pair<Vector<2>, double> *add_circle(Vector<2> center, double radius) {
    circles_.emplace_back(std::pair<Vector<2>, double>(center, radius));
    return &(circles_.back());
  }

  void Draw(RenderInterface *render, double w, double h) {
    render->Translate(w / 2.0, h / 2.0);
    render->SetSourceRGB(color.r / 255.0, color.g / 255.0, color.b / 255.0);
    for (const auto &circle : circles_) {
      render->Circle(scale * circle.first.x(), -scale * circle.first.y(),
                     scale * circle.second);
      render->Stroke();
    }
  }

  // empting the list will blank the whole overlay
  void Reset() { circles_.clear(); }

 private:
  // circles in this overlay
  std::vector<std::pair<Vector<2>, double>> circles_;
};

}  // vision
}  // aos

#endif  // _AOS_VISION_IMAGE_DEBUG_OVERLAY_H_
