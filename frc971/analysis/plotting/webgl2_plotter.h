#ifndef FRC971_ANALYSIS_PLOTTING_WEBGL2_PLOTTER_H_
#define FRC971_ANALYSIS_PLOTTING_WEBGL2_PLOTTER_H_

#include <vector>

#include <Eigen/Dense>
#define GL_GLEXT_PROTOTYPES
#include <GLES3/gl3.h>
#include <GLES3/gl2ext.h>
#include <GLES3/gl32.h>

namespace frc971 {
namespace plotting {

struct Color {
  float r;
  float g;
  float b;
};

class Line {
 public:
  virtual ~Line() {}
  virtual void SetPoints(const std::vector<Eigen::Vector2d> &pts) = 0;
  virtual void SetColor(const Color &color) = 0;
  virtual void Draw() = 0;
  virtual Eigen::Vector2d MaxValues() const = 0;
  virtual Eigen::Vector2d MinValues() const = 0;
  virtual void SetLineWidth(const float width) = 0;
  virtual void SetPointSize(const float point_size) = 0;
  virtual bool HasUpdate() = 0;
};

// TODO(james): Actually do something with this interface; originally, I'd meant
// to look at writing some tests, but right now it's just extra boilerplate.
class Plotter {
 public:
  virtual Line *AddLine() = 0;
  virtual void SetScale(const Eigen::Vector2d &scale) = 0;
  virtual Eigen::Vector2d GetScale() const = 0;
  virtual void SetOffset(const Eigen::Vector2d &offset) = 0;
  virtual Eigen::Vector2d GetOffset() const = 0;
  virtual void Redraw() = 0;
  virtual Eigen::Vector2d MaxValues() const = 0;
  virtual Eigen::Vector2d MinValues() const = 0;
  virtual void ClearZoomRectangle() = 0;
  virtual void SetZoomRectangle(const Eigen::Vector2d &corner1,
                                const Eigen::Vector2d &corner2) = 0;
  virtual void RecordState() = 0;
  virtual void Undo() = 0;
};

class WebglCanvasPlotter : public Plotter {
 public:
  WebglCanvasPlotter(const std::string &canvas_id,
                     GLuint attribute_location = 0);
  Line *AddLine() override;
  void SetScale(const Eigen::Vector2d &scale) override;
  Eigen::Vector2d GetScale() const override;
  void SetOffset(const Eigen::Vector2d &offset) override;
  Eigen::Vector2d GetOffset() const override;
  void Redraw() override;
  Eigen::Vector2d MaxValues() const override;
  Eigen::Vector2d MinValues() const override;
  void ClearZoomRectangle() override;
  void SetZoomRectangle(const Eigen::Vector2d &corner1,
                        const Eigen::Vector2d &corner2) override;
  void RecordState() override;
  void Undo() override;

 private:
  std::vector<std::unique_ptr<Line>> lines_;
  std::unique_ptr<Line> zoom_rectangle_;
  Eigen::Vector2d scale_{1.0, 1.0};
  Eigen::Vector2d offset_{0.0, 0.0};
  std::vector<Eigen::Vector2d> old_scales_;
  std::vector<Eigen::Vector2d> old_offsets_;
  Eigen::Vector2d last_scale_{1.0, 1.0};
  Eigen::Vector2d last_offset_{0.0, 0.0};
  GLuint program_;
  GLuint scale_uniform_location_;
  GLuint offset_uniform_location_;
  GLuint gl_buffer_;
};

}  // namespace plotting
}  // namespace frc971
#endif  // FRC971_ANALYSIS_PLOTTING_WEBGL2_PLOTTER_H_
