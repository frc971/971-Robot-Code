#ifndef AOS_VISION_DEBUG_DEBUG_WINDOW_H_
#define AOS_VISION_DEBUG_DEBUG_WINDOW_H_

#include <cairo.h>
#include <functional>
#include "aos/vision/debug/overlay.h"
#include "aos/vision/image/image_types.h"

namespace aos {
namespace vision {

// Implement Cairo version of RenderInterface.
class CairoRender : public RenderInterface {
 public:
  explicit CairoRender(cairo_t *cr, double text_scale)
      : cr_(cr), text_scale_(text_scale) {}
  virtual ~CairoRender() {}

  void Translate(double x, double y) override { cairo_translate(cr_, x, y); }

  void SetSourceRGB(double r, double g, double b) override {
    cairo_set_source_rgb(cr_, r, g, b);
  }

  void MoveTo(double x, double y) override { cairo_move_to(cr_, x, y); }

  void LineTo(double x, double y) override { cairo_line_to(cr_, x, y); }

  void Circle(double x, double y, double r) override {
    cairo_arc(cr_, x, y, r, 0.0, 2 * M_PI);
  }

  void Stroke() override { cairo_stroke(cr_); }

  void Text(int x, int y, int text_x, int text_y,
            const std::string &text) override;

 private:
  cairo_t *cr_;
  double text_scale_ = 1.0;
};

// Simple debug view window.
class DebugWindow {
 public:
  struct Internals;
  explicit DebugWindow(bool flip);
  ~DebugWindow();
  // Explicit redraw queuing (Will not double-queue).
  void Redraw();

  // This will resize the window as well as updating to draw from the
  // (not owned) ptr. When you change ptr, you should call Redraw();
  void UpdateImage(ImagePtr ptr);

  // Sets up the window to draw a list of overlays.
  // See overlay.h for more info.
  void SetOverlays(std::vector<OverlayBase *> *overlay);

  void AddOverlay(OverlayBase *overlay);
  void AddOverlays(const std::vector<OverlayBase *> &overlays) {
    for (auto *overlay : overlays) {
      AddOverlay(overlay);
    }
  }

  // Resizes the window.
  void SetScale(double scale_factor);

  // Move window.
  void MoveTo(int x, int y);

  // Set to change the key_press behaviour.
  // The argument type is a constant that looks like: GDK_KEY_#{key_val_name}
  std::function<void(uint32_t)> key_press_event;

 private:
  bool shown_yet_ = false;
  double scale_factor = 1.0;
  int window_width_ = 100;
  int window_height_ = 100;
  std::unique_ptr<Internals> self;
};

}  // namespace vision
}  // namespace aos

#endif  // AOS_VISION_DEBUG_DEBUG_WINDOW_H_
