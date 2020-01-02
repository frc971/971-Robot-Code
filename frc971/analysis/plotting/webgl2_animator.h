#ifndef FRC971_ANALYSIS_PLOTTING_WEBGL2_ANIMATOR_H_
#define FRC971_ANALYSIS_PLOTTING_WEBGL2_ANIMATOR_H_

#include <Eigen/Dense>
#include <emscripten/emscripten.h>
#include <emscripten/html5.h>

#include "frc971/analysis/plotting/webgl2_plotter.h"

namespace frc971 {
namespace plotting {

// TODO(james): Write some tests for this class. It shouldn't be too hard to
// abstract out all the direct emscripten calls. Mostly it's just some
// initialization at the moment.
class Animator {
 public:
  Animator(const char *canvas_target);

  Plotter *plotter() { return &plotter_; }

 private:
  Eigen::Vector2d MouseCanvasLocation(const EmscriptenMouseEvent &mouse_event);

  Eigen::Vector2d CanvasToPlotLocation(const Eigen::Vector2d &canvas_loc);

  void PrintZoom();

  void PrintPosition(const EmscriptenMouseEvent &mouse_event);

  void HandleMouseUp(const EmscriptenMouseEvent &mouse_event);

  void HandleMouseDown(const EmscriptenMouseEvent &mouse_event);

  void HandleMouseMove(const EmscriptenMouseEvent &mouse_event);

  void HandleMouseEnter(const EmscriptenMouseEvent &mouse_event);

  void SetZoomCorners(const Eigen::Vector2d &c1, const Eigen::Vector2d &c2);

  void SetFilteredZoom(Eigen::Vector2d scale, Eigen::Vector2d offset);

  void ResetView();

  static int Redraw(double time_ms, void *data);
  static int KeyboardCallback(int event_type,
                              const EmscriptenKeyboardEvent *key_event,
                              void *data);
  static int WheelCallback(int event_type,
                           const EmscriptenWheelEvent *wheel_event, void *data);
  static int MouseCallback(int event_type,
                           const EmscriptenMouseEvent *mouse_event, void *data);

  int canvas_width_ = 0.0;
  int canvas_height_ = 0.0;

  // Location, in canvas coordinates of the last left click mouse-down event.
  Eigen::Vector2d mouse_down_location_{0, 0};

  // True if the user is currently dragging their mouse to zoom to a rectangle.
  // This is used to (a) determine whether we should subsequently execute the
  // zoom when the user releases the mouse and (b) to know when to draw a
  // rectangle indicating where the user is zooming to.
  bool doing_rectangle_zoom_ = false;

  // The last location of the mouse when panning, so that we can calculate
  // exactly how much the mouse has moved since the last mouse-move callback.
  Eigen::Vector2d last_pan_mouse_location_{0, 0};

  // Whether the "x" or "y" key is currently pressed on the keyboard.
  bool x_pressed_ = false;
  bool y_pressed_ = false;

  WebglCanvasPlotter plotter_;
};

}  // namespace plotting
}  // namespace frc971
#endif  // FRC971_ANALYSIS_PLOTTING_WEBGL2_ANIMATOR_H_
