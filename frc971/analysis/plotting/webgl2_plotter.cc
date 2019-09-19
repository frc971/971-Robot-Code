#include "frc971/analysis/plotting/webgl2_plotter.h"

#include <assert.h>
#include <stdlib.h>

#include <iostream>

#include <emscripten/emscripten.h>
#include <emscripten/html5.h>

namespace frc971 {
namespace plotting {

namespace {
// Shader and program construction taken from examples at
// https://github.com/emscripten-core/emscripten/blob/incoming/tests/webgl2_draw_packed_triangle.c
GLuint compile_shader(GLenum shaderType, const char *src) {
  GLuint shader = glCreateShader(shaderType);
  glShaderSource(shader, 1, &src, nullptr);
  glCompileShader(shader);

  GLint isCompiled = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &isCompiled);
  if (!isCompiled) {
    GLint maxLength = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &maxLength);
    char *buf = (char *)malloc(maxLength + 1);
    glGetShaderInfoLog(shader, maxLength, &maxLength, buf);
    printf("%s\n", buf);
    free(buf);
    return 0;
  }

  return shader;
}

GLuint create_program(GLuint vertexShader, GLuint fragmentShader,
                      GLuint attribute_location) {
  GLuint program = glCreateProgram();
  glAttachShader(program, vertexShader);
  glAttachShader(program, fragmentShader);
  glBindAttribLocation(program, attribute_location, "apos");
  glLinkProgram(program);
  return program;
}

GLuint CreateLineProgram(GLuint attribute_location) {
  // Create a shader program which will take in:
  // -A series of points to plot
  // -Scale/offset parameters for choosing how to zoom/pan
  // -Point size/color information
  //
  // The vertex shader then takes in the series of points (apos) and
  // transforms the points by the scale/offset to determine their on-screen
  // position. We aren't doing any funny with 3D or perspective, so we leave
  // the z and w components untouched.
  //
  // We set the color of the line in the fragment shader.
  const char vertex_shader[] =
    "#version 100\n"
    "attribute vec2 apos;"
    "uniform vec2 scale;"
    "uniform vec2 offset;"
    "uniform float point_size;"
    "void main() {"
      "gl_Position.xy = apos.xy * scale.xy + offset.xy;"
      "gl_Position.z = 0.0;"
      "gl_Position.w = 1.0;"
      "gl_PointSize = point_size;"
    "}";
  GLuint vs = compile_shader(GL_VERTEX_SHADER, vertex_shader);

  const char fragment_shader[] =
    "#version 100\n"
    "precision lowp float;"
    "uniform vec4 color;"
    "void main() {"
      "gl_FragColor = color;"
    "}";
  GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fragment_shader);
  return create_program(vs, fs, attribute_location);
}

const Eigen::Vector2d Vector2dInfinity() {
  return Eigen::Vector2d::Ones() * std::numeric_limits<double>::infinity();
}

}  // namespace

class WebglLine : public Line {
 public:
  WebglLine(GLuint program, size_t buffer_size = 1000000)
      : color_uniform_location_(glGetUniformLocation(program, "color")),
        point_size_uniform_location_(
            glGetUniformLocation(program, "point_size")),
        line_size_(0) {}
  virtual ~WebglLine() {}
  void SetPoints(const std::vector<Eigen::Vector2d> &pts) override {
    updated_ = true;
    max_values_ = -Vector2dInfinity();
    min_values_ = Vector2dInfinity();
    line_size_ = 0;
    buffer_.clear();
    for (const auto &pt : pts) {
      buffer_.push_back(pt.x());
      buffer_.push_back(pt.y());
      max_values_ = max_values_.cwiseMax(pt);
      min_values_ = min_values_.cwiseMin(pt);
      ++line_size_;
    }
  }
  void Draw() override {
    updated_ = false;
    if (buffer_.empty()) {
      return;
    }
    // TODO(james): Flushing and rewriting the buffer on every line draw seems
    // like it should be inefficient, but in practice it seems to actually be
    // fine for the amounts of data that we are dealing with (i.e., doing a few
    // tens of MB of copies at the most is not actually that expensive).
    glBufferData(GL_ARRAY_BUFFER, buffer_.size() * sizeof(float),
                 buffer_.data(), GL_STATIC_DRAW);
    glUniform4f(color_uniform_location_, color_.r, color_.g, color_.b, 1.0);
    glUniform1f(point_size_uniform_location_, point_size_);
    if (point_size_ != 0) {
      glDrawArrays(GL_POINTS, 0, line_size_);
    }
    if (line_width_ != 0) {
      glDrawArrays(GL_LINE_STRIP, 0, line_size_);
    }
    assert(GL_NO_ERROR == glGetError() && "glDrawArray failed");
  }
  void SetColor(const Color &color) override {
    updated_ = true;
    color_ = color;
  }

  Eigen::Vector2d MaxValues() const override { return max_values_; }
  Eigen::Vector2d MinValues() const override { return min_values_; }

  void SetLineWidth(const float width) override {
    updated_ = true;
    line_width_ = width;
  }
  void SetPointSize(const float point_size) override {
    updated_ = true;
    point_size_ = point_size;
  }

  bool HasUpdate() override { return updated_; }

 private:
  const GLuint color_uniform_location_;
  const GLuint point_size_uniform_location_;
  std::vector<float> buffer_;
  size_t line_size_;
  Color color_;
  Eigen::Vector2d max_values_ = -Vector2dInfinity();
  Eigen::Vector2d min_values_ = Vector2dInfinity();
  float line_width_ = 1.0;
  float point_size_ = 3.0;
  bool updated_ = true;
};

WebglCanvasPlotter::WebglCanvasPlotter(const std::string &canvas_id,
                                       GLuint attribute_location)  {
  EmscriptenWebGLContextAttributes attr;
  emscripten_webgl_init_context_attributes(&attr);
  assert(attr.antialias && "Antialiasing should be enabled by default.");
  attr.majorVersion = 2;
  EMSCRIPTEN_WEBGL_CONTEXT_HANDLE ctx = emscripten_webgl_create_context("#canvas", &attr);
  assert(ctx && "Failed to create WebGL2 context");
  emscripten_webgl_make_context_current(ctx);

  program_ = CreateLineProgram(attribute_location);
  scale_uniform_location_ = glGetUniformLocation(program_, "scale");
  offset_uniform_location_ = glGetUniformLocation(program_, "offset");

  glGenBuffers(1, &gl_buffer_);
  glUseProgram(program_);
  glBindBuffer(GL_ARRAY_BUFFER, gl_buffer_);
  glVertexAttribPointer(attribute_location, 2, GL_FLOAT, GL_FALSE, 8, 0);
  glEnableVertexAttribArray(attribute_location);

  zoom_rectangle_ = std::make_unique<WebglLine>(program_);
  zoom_rectangle_->SetColor({.r = 1.0, .g = 1.0, .b = 1.0});
  zoom_rectangle_->SetLineWidth(2.0);
  zoom_rectangle_->SetPointSize(0.0);
}

Line *WebglCanvasPlotter::AddLine() {
  lines_.push_back(std::make_unique<WebglLine>(program_));
  return lines_.back().get();
}
void WebglCanvasPlotter::Undo() {
  if (old_scales_.empty() || old_offsets_.empty()) {
    return;
  }
  scale_ = old_scales_.back();
  old_scales_.pop_back();
  offset_ = old_offsets_.back();
  old_offsets_.pop_back();
}

void WebglCanvasPlotter::RecordState() {
  old_scales_.push_back(scale_);
  old_offsets_ .push_back(offset_);
}

void WebglCanvasPlotter::SetScale(const Eigen::Vector2d &scale) {
  scale_ = scale;
}

Eigen::Vector2d WebglCanvasPlotter::GetScale() const {
  return scale_;
}

void WebglCanvasPlotter::SetOffset(const Eigen::Vector2d &offset) {
  offset_ = offset;
}

Eigen::Vector2d WebglCanvasPlotter::GetOffset() const {
  return offset_;
}

void WebglCanvasPlotter::Redraw() {
  const bool scaling_update = last_scale_ != scale_ || last_offset_ != offset_;
  bool data_update = zoom_rectangle_->HasUpdate();
  for (const auto &line : lines_) {
    data_update = line->HasUpdate() || data_update;
  }
  if (!scaling_update && !data_update) {
    return;
  }
  glUseProgram(program_);
  glBindBuffer(GL_ARRAY_BUFFER, gl_buffer_);
  glUniform2f(scale_uniform_location_, scale_.x(), scale_.y());
  glUniform2f(offset_uniform_location_, offset_.x(), offset_.y());
  for (const auto &line : lines_) {
    line->Draw();
  }
  zoom_rectangle_->Draw();
  last_scale_ = scale_;
  last_offset_ = offset_;
}

Eigen::Vector2d WebglCanvasPlotter::MaxValues() const {
  Eigen::Vector2d max = -Vector2dInfinity();
  for (const auto &line : lines_) {
    max = max.cwiseMax(line->MaxValues());
  }
  return max;
}
Eigen::Vector2d WebglCanvasPlotter::MinValues() const {
  Eigen::Vector2d min = Vector2dInfinity();
  for (const auto &line : lines_) {
    min = min.cwiseMin(line->MinValues());
  }
  return min;
}

void WebglCanvasPlotter::ClearZoomRectangle() {
  zoom_rectangle_->SetPoints({});
}


void WebglCanvasPlotter::SetZoomRectangle(const Eigen::Vector2d &corner1,
                                          const Eigen::Vector2d &corner2) {
  zoom_rectangle_->SetPoints({corner1,
                              {corner1.x(), corner2.y()},
                              corner2,
                              {corner2.x(), corner1.y()},
                              corner1});
}

}  // namespace plotting
}  // namespace frc971
