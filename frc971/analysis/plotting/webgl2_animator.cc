#include "frc971/analysis/plotting/webgl2_animator.h"

namespace frc971 {
namespace plotting {

namespace {
struct Button {
  bool IsTransition(const EmscriptenMouseEvent &mouse_event) {
    return mouse_event.button == transition_number_;
  }
  bool IsPressed(const EmscriptenMouseEvent &mouse_event) {
    return mouse_event.buttons & (1 << pressed_index_);
  }
  const size_t transition_number_;
  const size_t pressed_index_;
};
constexpr Button kLeftButton() { return {0, 0}; }
//constexpr Button kMiddleButton() { return {1, 2}; }
constexpr Button kRightButton() { return {2, 1}; }

constexpr Button kPanButton() { return kLeftButton(); }
constexpr Button kZoomButton() { return kRightButton(); }
}  // namespace

Animator::Animator(const char *canvas_target) : plotter_(canvas_target) {
  // TODO(james): Write a proper CHECK macro or figure out how to import glog.
  // Importing glog is a bit of a pain, since it seems to assume all sorts of
  // things that don't really apply on the web.
  assert(EMSCRIPTEN_RESULT_SUCCESS ==
         emscripten_get_canvas_element_size(canvas_target, &canvas_width_,
                                            &canvas_height_));
  assert(EMSCRIPTEN_RESULT_SUCCESS ==
         emscripten_set_mousemove_callback("#canvas", this, true,
                                           &Animator::MouseCallback));
  assert(EMSCRIPTEN_RESULT_SUCCESS ==
         emscripten_set_click_callback("#canvas", this, true,
                                       &Animator::MouseCallback));
  assert(EMSCRIPTEN_RESULT_SUCCESS ==
         emscripten_set_mousedown_callback("#canvas", this, true,
                                           &Animator::MouseCallback));
  assert(EMSCRIPTEN_RESULT_SUCCESS ==
         emscripten_set_mouseup_callback("#canvas", this, true,
                                         &Animator::MouseCallback));
  assert(EMSCRIPTEN_RESULT_SUCCESS ==
         emscripten_set_mouseleave_callback("#canvas", this, true,
                                            &Animator::MouseCallback));
  assert(EMSCRIPTEN_RESULT_SUCCESS ==
         emscripten_set_mouseenter_callback("#canvas", this, true,
                                            &Animator::MouseCallback));
  assert(EMSCRIPTEN_RESULT_SUCCESS ==
         emscripten_set_dblclick_callback("#canvas", this, true,
                                          &Animator::MouseCallback));
  assert(EMSCRIPTEN_RESULT_SUCCESS ==
         emscripten_set_wheel_callback("#canvas", this, true,
                                       &Animator::WheelCallback));
  assert(EMSCRIPTEN_RESULT_SUCCESS ==
         emscripten_set_keypress_callback("#document", this, true,
                                          &Animator::KeyboardCallback));
  assert(EMSCRIPTEN_RESULT_SUCCESS ==
         emscripten_set_keydown_callback("#document", this, true,
                                         &Animator::KeyboardCallback));
  assert(EMSCRIPTEN_RESULT_SUCCESS ==
         emscripten_set_keyup_callback("#document", this, true,
                                       &Animator::KeyboardCallback));
  emscripten_request_animation_frame_loop(&Animator::Redraw, this);
}

Eigen::Vector2d Animator::MouseCanvasLocation(
    const EmscriptenMouseEvent &mouse_event) {
  return {mouse_event.canvasX * 2.0 / canvas_width_ - 1.0,
          -mouse_event.canvasY * 2.0 / canvas_height_ + 1.0};
}

Eigen::Vector2d Animator::CanvasToPlotLocation(
    const Eigen::Vector2d &canvas_loc) {
  return (canvas_loc - plotter_.GetOffset()).cwiseQuotient(plotter_.GetScale());
}

void Animator::PrintZoom() {
  const Eigen::Vector2d upper_right = CanvasToPlotLocation({1.0, 1.0});
  const Eigen::Vector2d lower_left = CanvasToPlotLocation({-1.0, -1.0});
  printf("X range is [%f, %f]; Y range is [%f, %f]\n", lower_left.x(),
         upper_right.x(), lower_left.y(), upper_right.y());
}

void Animator::PrintPosition(const EmscriptenMouseEvent &mouse_event) {
  const Eigen::Vector2d mouse_pos =
      CanvasToPlotLocation(MouseCanvasLocation(mouse_event));
  printf("Mouse position: (%f, %f)\n", mouse_pos.x(), mouse_pos.y());
}

void Animator::HandleMouseUp(const EmscriptenMouseEvent &mouse_event) {
  if (!kZoomButton().IsTransition(mouse_event)) {
    return;
  }
  // We aborted the zoom early for some reason and so shouldn't execute on it:
  if (!doing_rectangle_zoom_) {
    return;
  }
  const Eigen::Vector2d mouse_up_location = MouseCanvasLocation(mouse_event);
  doing_rectangle_zoom_ = false;
  plotter_.ClearZoomRectangle();
  // The user probably didn't mean to zoom on that click...
  if ((mouse_up_location - mouse_down_location_).cwiseAbs().minCoeff() < 1e-3) {
    return;
  }
  const Eigen::Vector2d mouse_up_plot_location =
      CanvasToPlotLocation(mouse_up_location);
  const Eigen::Vector2d mouse_down_plot_location =
      CanvasToPlotLocation(mouse_down_location_);
  SetZoomCorners(mouse_down_plot_location, mouse_up_plot_location);
}

void Animator::HandleMouseDown(const EmscriptenMouseEvent &mouse_event) {
  if (kZoomButton().IsTransition(mouse_event)) {
    mouse_down_location_ = MouseCanvasLocation(mouse_event);
    doing_rectangle_zoom_ = true;
  } else if (kPanButton().IsTransition(mouse_event)) {
    last_pan_mouse_location_ = MouseCanvasLocation(mouse_event);
  }
}

void Animator::HandleMouseMove(const EmscriptenMouseEvent &mouse_event) {
  const Eigen::Vector2d mouse_location = MouseCanvasLocation(mouse_event);
  if (kPanButton().IsPressed(mouse_event)) {
    SetFilteredZoom(plotter_.GetScale(), plotter_.GetOffset() + mouse_location -
                                            last_pan_mouse_location_);
    last_pan_mouse_location_ = mouse_location;
  }
  if (doing_rectangle_zoom_) {
    Eigen::Vector2d c1 = CanvasToPlotLocation(mouse_down_location_);
    Eigen::Vector2d c2 = CanvasToPlotLocation(mouse_location);
    const Eigen::Vector2d upper_right = CanvasToPlotLocation({1.0, 1.0});
    const Eigen::Vector2d bottom_left = CanvasToPlotLocation({-1.0, -1.0});
    if (x_pressed_ && !y_pressed_) {
      c1.y() = upper_right.y();
      c2.y() = bottom_left.y();
    }
    if (y_pressed_ && !x_pressed_) {
      c1.x() = upper_right.x();
      c2.x() = bottom_left.x();
    }
    plotter_.SetZoomRectangle(c1, c2);
  }
}

void Animator::HandleMouseEnter(const EmscriptenMouseEvent &mouse_event) {
  // If the zoom button is unclicked and we were zooming, instantly finish the
  // rectangle zoom.
  if (doing_rectangle_zoom_ && !kZoomButton().IsPressed(mouse_event)) {
    plotter_.ClearZoomRectangle();
    doing_rectangle_zoom_ = false;
    // Round the current mouse location to the nearest of the four corners.
    // This is to ensure that a zoom that occurs when the use goes of the edge
    // of the screen actually goes right up to the edge of the canvas.
    // Technically, cwiseSign will return zero if you get the mouse enter
    // event to trigger with the mouse at the center of the screen, but that
    // seems unlikely.
    const Eigen::Vector2d canvas_corner =
        MouseCanvasLocation(mouse_event).cwiseSign();
    SetZoomCorners(CanvasToPlotLocation(mouse_down_location_),
                   CanvasToPlotLocation(canvas_corner));
  }
}

void Animator::SetZoomCorners(const Eigen::Vector2d &c1,
                              const Eigen::Vector2d &c2) {
  const Eigen::Vector2d scale = ((c2 - c1).cwiseAbs() / 2.0).cwiseInverse();
  const Eigen::Vector2d offset =
      Eigen::Vector2d::Ones() - scale.cwiseProduct(c2.cwiseMax(c1));
  SetFilteredZoom(scale, offset);
}

void Animator::SetFilteredZoom(Eigen::Vector2d scale, Eigen::Vector2d offset) {
  if (!x_pressed_ && y_pressed_) {
    scale.x() = plotter_.GetScale().x();
    offset.x() = plotter_.GetOffset().x();
  }
  if (!y_pressed_ && x_pressed_) {
    scale.y() = plotter_.GetScale().y();
    offset.y() = plotter_.GetOffset().y();
  }
  plotter_.RecordState();
  plotter_.SetScale(scale);
  plotter_.SetOffset(offset);
  PrintZoom();
}

void Animator::ResetView() {
  SetZoomCorners(plotter_.MinValues(), plotter_.MaxValues());
}

int Animator::Redraw(double time_ms, void *data) {
  Animator *state = reinterpret_cast<Animator *>(data);
  state->plotter_.Redraw();
  return 1;
}

int Animator::KeyboardCallback(int event_type,
                               const EmscriptenKeyboardEvent *key_event,
                               void *data) {
  Animator *state = reinterpret_cast<Animator *>(data);
  const bool key_is_pressed = event_type == EMSCRIPTEN_EVENT_KEYDOWN;
  if (strncmp(key_event->key, "x", 2) == 0) {
    state->x_pressed_ = key_is_pressed;
    return true;
  } else if (strncmp(key_event->key, "y", 2) == 0) {
    state->y_pressed_ = key_is_pressed;
    return true;
  } else if (strncmp(key_event->key, "z", 2) == 0 && key_event->ctrlKey) {
    if (event_type == EMSCRIPTEN_EVENT_KEYUP) {
      state->plotter_.Undo();
      state->PrintZoom();
    }
    return true;
  } else if (strncmp(key_event->key, "Escape", 7) == 0) {
    state->doing_rectangle_zoom_ = false;
    state->plotter_.ClearZoomRectangle();
    return true;
  }
  return false;
}

int Animator::WheelCallback(int event_type,
                            const EmscriptenWheelEvent *wheel_event,
                            void *data) {
  Animator *state = reinterpret_cast<Animator *>(data);
  assert(event_type == EMSCRIPTEN_EVENT_WHEEL);
  if (wheel_event->deltaMode == DOM_DELTA_PIXEL) {
    const Eigen::Vector2d canvas_pos =
        state->MouseCanvasLocation(wheel_event->mouse);
    constexpr double kWheelTuningScalar = 3.0;
    const double zoom =
        -kWheelTuningScalar * wheel_event->deltaY / state->canvas_height_;
    double zoom_scalar = 1.0 + std::abs(zoom);
    if (zoom < 0) {
      zoom_scalar = 1.0 / zoom_scalar;
    }
    const Eigen::Vector2d scale = state->plotter_.GetScale() * zoom_scalar;
    const Eigen::Vector2d offset = (1.0 - zoom_scalar) * canvas_pos +
                                   zoom_scalar * state->plotter_.GetOffset();
    state->SetFilteredZoom(scale, offset);
    return true;
  }
  return false;
}

int Animator::MouseCallback(int event_type,
                            const EmscriptenMouseEvent *mouse_event,
                            void *data) {
  Animator *state = reinterpret_cast<Animator *>(data);
  switch (event_type) {
    case EMSCRIPTEN_EVENT_CLICK:
      state->PrintZoom();
      state->PrintPosition(*mouse_event);
      return true;
      break;
    case EMSCRIPTEN_EVENT_DBLCLICK:
      state->ResetView();
      return true;
      break;
    case EMSCRIPTEN_EVENT_MOUSEDOWN:
      state->HandleMouseDown(*mouse_event);
      return true;
      break;
    case EMSCRIPTEN_EVENT_MOUSEUP:
      state->HandleMouseUp(*mouse_event);
      return true;
      break;
    case EMSCRIPTEN_EVENT_MOUSEMOVE:
      state->HandleMouseMove(*mouse_event);
      return true;
      break;
    case EMSCRIPTEN_EVENT_MOUSEENTER:
      state->HandleMouseEnter(*mouse_event);
      return true;
      break;
  }
  return false;
}

}  // namespace plotting
}  // namespace frc971
