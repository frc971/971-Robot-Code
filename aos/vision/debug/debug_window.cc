#include "aos/vision/debug/debug_window.h"

#include <gdk-pixbuf/gdk-pixbuf.h>
#include <gtk/gtk.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <functional>
#include <memory>
#include <vector>

#include "aos/vision/image/image_types.h"

namespace aos {
namespace vision {

template <typename T, gboolean (T::*DrawMethod)(cairo_t *cr)>
gboolean DrawCallback(GtkWidget *, cairo_t *cr, gpointer data) {
  return ((*((T *)data)).*DrawMethod)(cr);
}

template <typename T, gboolean (T::*DrawMethod)(cairo_t *cr)>
void g_draw_signal_connect(GtkWidget *widget, T *obj) {
  gboolean (*fnptr)(GtkWidget *, cairo_t *, gpointer) =
      DrawCallback<T, DrawMethod>;
  g_signal_connect(widget, "draw", G_CALLBACK(fnptr), obj);
}

struct DebugWindow::Internals {
  Internals(bool flip) : flip_(flip) {}

  gboolean Draw(cairo_t *cr) {
    needs_draw = false;
    cairo_scale(cr, scale_factor, scale_factor);
    if (pixbuf != nullptr) {
      cairo_save(cr);
      if (flip_) {
        cairo_translate(cr, ptr.fmt().w, ptr.fmt().h);
        cairo_scale(cr, -1, -1);
      }
      gdk_cairo_set_source_pixbuf(cr, pixbuf, 0.0, 0.0);
      cairo_paint(cr);
      cairo_restore(cr);
    }

    int w = ptr.fmt().w;
    int h = ptr.fmt().h;
    if (overlays) {
      for (const auto &ov : *overlays) {
        cairo_save(cr);
        CairoRender render(cr, 1.0 / scale_factor);
        ov->Draw(&render, w, h);
        cairo_restore(cr);
      }
    }

    for (const auto &ov : overlays_internal_) {
      cairo_save(cr);
      CairoRender render(cr, 1.0 / scale_factor);
      ov->Draw(&render, w, h);
      cairo_restore(cr);
    }

    return FALSE;
  }

  GdkPixbuf *pixbuf = nullptr;
  GtkWidget *drawing_area = nullptr;
  ImagePtr ptr;
  bool needs_draw = true;
  GtkWidget *window;
  std::vector<OverlayBase *> *overlays = nullptr;
  std::vector<OverlayBase *> overlays_internal_;
  double scale_factor;

  // flip the image rows on drawing
  bool flip_ = false;

  // clear per frame
  bool clear_per_frame_ = true;
};

void DebugWindow::SetOverlays(std::vector<OverlayBase *> *overlays) {
  self->overlays = overlays;
}

void DebugWindow::AddOverlay(OverlayBase *overlay) {
  self->overlays_internal_.push_back(overlay);
}

void DebugWindow::Redraw() {
  if (!self->needs_draw) {
    gtk_widget_queue_draw(self->drawing_area);
    self->needs_draw = true;
  }
}

void DebugWindow::UpdateImage(ImagePtr ptr) {
  if (ptr.data() != self->ptr.data()) {
    int w = ptr.fmt().w;
    int h = ptr.fmt().h;
    self->pixbuf = gdk_pixbuf_new_from_data(
        (const unsigned char *)ptr.data(), GDK_COLORSPACE_RGB, FALSE, 8,
        ptr.fmt().w, ptr.fmt().h, 3 * ptr.fmt().w, NULL, NULL);
    self->ptr = ptr;

    gtk_window_set_default_size(GTK_WINDOW(self->window), w * scale_factor,
                                h * scale_factor);

    gtk_widget_set_size_request(self->drawing_area, w * scale_factor,
                                h * scale_factor);
    window_height_ = h;
    window_width_ = w;
  }
  if (!shown_yet_) {
    gtk_widget_show_all(self->window);
    shown_yet_ = true;
  }
}

void DebugWindow::MoveTo(int x, int y) {
  gtk_window_move(GTK_WINDOW(self->window), x, y);
}

void DebugWindow::SetScale(double scale_factor_inp) {
  int w = window_width_;
  int h = window_height_;

  scale_factor = scale_factor_inp;
  self->scale_factor = scale_factor;

  gtk_window_resize(GTK_WINDOW(self->window), w * scale_factor,
                    h * scale_factor);

  gtk_widget_set_size_request(self->drawing_area, w * scale_factor,
                              h * scale_factor);
}

gboolean debug_viewer_key_press_event(GtkWidget * /*widget*/,
                                      GdkEventKey *event, gpointer user_data) {
  auto &key_press_cb =
      reinterpret_cast<DebugWindow *>(user_data)->key_press_event;
  if (key_press_cb) key_press_cb(event->keyval);
  return FALSE;
}

DebugWindow::DebugWindow(bool flip) : self(new Internals(flip)) {
  self->scale_factor = scale_factor;
  GtkWidget *window;
  auto drawing_area = self->drawing_area = gtk_drawing_area_new();
  gtk_widget_set_size_request(drawing_area, window_width_ * scale_factor,
                              window_height_ * scale_factor);
  gtk_widget_add_events(drawing_area, GDK_KEY_PRESS_MASK);

  g_draw_signal_connect<DebugWindow::Internals, &DebugWindow::Internals::Draw>(
      drawing_area, self.get());

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  self->window = window;
  g_signal_connect(window, "key-press-event",
                   G_CALLBACK(debug_viewer_key_press_event), this);
  gtk_window_set_title(GTK_WINDOW(window), "Window");
  gtk_window_set_default_size(GTK_WINDOW(window), window_width_ * scale_factor,
                              window_height_ * scale_factor);

  gtk_container_add(GTK_CONTAINER(window), drawing_area);
}
DebugWindow::~DebugWindow() {}

void CairoRender::Text(int x, int y, int /*text_x*/, int /*text_y*/,
                       const std::string &text) {
  cairo_save(cr_);
  cairo_move_to(cr_, x, y);
  cairo_scale(cr_, text_scale_, text_scale_);

  auto *pango_lay = pango_cairo_create_layout(cr_);
  pango_layout_set_text(pango_lay, text.data(), text.size());
  pango_cairo_show_layout(cr_, pango_lay);
  g_object_unref(pango_lay);
  cairo_restore(cr_);
}

}  // namespace vision
}  // namespace aos
