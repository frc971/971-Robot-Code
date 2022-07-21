#include "frc971/analysis/in_process_plotter.h"

#include "aos/configuration.h"

namespace frc971 {
namespace analysis {

namespace {
const char *kDataPath = "frc971/analysis/cpp_plot";
const char *kConfigPath = "frc971/analysis/plotter.json";
}  // namespace

Plotter::Plotter()
    : config_(aos::configuration::ReadConfig(kConfigPath)),
      event_loop_factory_(&config_.message()),
      event_loop_(event_loop_factory_.MakeEventLoop("plotter")),
      plot_sender_(event_loop_->MakeSender<Plot>("/analysis")),
      web_proxy_(event_loop_.get(), aos::web_proxy::StoreHistory::kYes, -1),
      builder_(plot_sender_.MakeBuilder()) {
  web_proxy_.SetDataPath(kDataPath);
  event_loop_->SkipTimingReport();

  color_wheel_.emplace_back(ColorWheelColor{.name = "red", .color = {1, 0, 0}});
  color_wheel_.emplace_back(
      ColorWheelColor{.name = "green", .color = {0, 1, 0}});
  color_wheel_.emplace_back(
      ColorWheelColor{.name = "purple", .color = {0.54, 0.3, 0.75}});
  color_wheel_.emplace_back(
      ColorWheelColor{.name = "blue", .color = {0, 0, 1}});
  color_wheel_.emplace_back(
      ColorWheelColor{.name = "yellow", .color = {1, 1, 0}});
  color_wheel_.emplace_back(
      ColorWheelColor{.name = "teal", .color = {0, 1, 1}});
  color_wheel_.emplace_back(
      ColorWheelColor{.name = "pink", .color = {1, 0, 1}});
  color_wheel_.emplace_back(
      ColorWheelColor{.name = "orange", .color = {1, 0.6, 0}});
  color_wheel_.emplace_back(
      ColorWheelColor{.name = "brown", .color = {0.6, 0.3, 0}});
  color_wheel_.emplace_back(
      ColorWheelColor{.name = "white", .color = {1, 1, 1}});
}

void Plotter::Spin() { event_loop_factory_.Run(); }

void Plotter::Title(std::string_view title) {
  title_ = builder_.fbb()->CreateString(title);
}

void Plotter::AddFigure(std::string_view title, double width, double height) {
  MaybeFinishFigure();

  if (!title.empty()) {
    figure_title_ = builder_.fbb()->CreateString(title);
  }

  // For positioning, just stack figures vertically.
  auto position_builder = builder_.MakeBuilder<Position>();
  position_builder.add_top(next_top_);
  position_builder.add_left(0);
  position_builder.add_width(width);
  position_builder.add_height(height);
  position_ = position_builder.Finish();

  next_top_ += height;
}

void Plotter::XLabel(std::string_view label) {
  xlabel_ = builder_.fbb()->CreateString(label);
}

void Plotter::YLabel(std::string_view label) {
  ylabel_ = builder_.fbb()->CreateString(label);
}

void Plotter::AddLine(const std::vector<double> &x,
                      const std::vector<double> &y, LineOptions options) {
  CHECK_EQ(x.size(), y.size());
  CHECK(!position_.IsNull())
      << "You must call AddFigure() before calling AddLine().";

  flatbuffers::Offset<flatbuffers::String> label_offset;
  if (!options.label.empty()) {
    label_offset = builder_.fbb()->CreateString(options.label);
  }

  std::vector<Point> points;
  for (size_t ii = 0; ii < x.size(); ++ii) {
    points.emplace_back(x[ii], y[ii]);
  }
  const flatbuffers::Offset<flatbuffers::Vector<const Point *>> points_offset =
      builder_.fbb()->CreateVectorOfStructs(points);

  const Color *color;
  if (options.color.empty()) {
    color = &color_wheel_.at(color_wheel_position_).color;
    color_wheel_position_ = (color_wheel_position_ + 1) % color_wheel_.size();
  } else {
    auto it = std::find_if(
        color_wheel_.begin(), color_wheel_.end(),
        [options_color = options.color](const ColorWheelColor &color) {
          return color.name == options_color;
        });
    CHECK(it != color_wheel_.end()) << ": Failed to find " << options.color;
    color = &(it->color);
  }

  LineStyle::Builder style_builder = builder_.MakeBuilder<LineStyle>();
  if (options.line_style.find('*') != options.line_style.npos) {
    style_builder.add_point_size(options.point_size);
  } else {
    style_builder.add_point_size(0.0);
  }
  style_builder.add_draw_line(options.line_style.find('-') !=
                              options.line_style.npos);
  const flatbuffers::Offset<LineStyle> style_offset = style_builder.Finish();

  auto line_builder = builder_.MakeBuilder<Line>();
  line_builder.add_label(label_offset);
  line_builder.add_points(points_offset);
  line_builder.add_color(color);
  line_builder.add_style(style_offset);
  lines_.push_back(line_builder.Finish());
}

void Plotter::MaybeFinishFigure() {
  if (!lines_.empty()) {
    const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Line>>>
        lines_offset = builder_.fbb()->CreateVector(lines_);
    auto figure_builder = builder_.MakeBuilder<Figure>();
    figure_builder.add_title(figure_title_);
    figure_builder.add_position(position_);
    figure_builder.add_lines(lines_offset);
    figure_builder.add_xlabel(xlabel_);
    figure_builder.add_share_x_axis(share_x_axis_);
    figure_builder.add_ylabel(ylabel_);
    figures_.push_back(figure_builder.Finish());
  }
  lines_.clear();
  figure_title_.o = 0;
  xlabel_.o = 0;
  ylabel_.o = 0;
  position_.o = 0;
  share_x_axis_ = false;
  color_wheel_position_ = 0;
}

void Plotter::Publish() {
  MaybeFinishFigure();
  const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Figure>>>
      figures_offset = builder_.fbb()->CreateVector(figures_);

  auto plot_builder = builder_.MakeBuilder<Plot>();
  plot_builder.add_title(title_);
  plot_builder.add_figures(figures_offset);

  CHECK_EQ(builder_.Send(plot_builder.Finish()), aos::RawSender::Error::kOk);

  builder_ = plot_sender_.MakeBuilder();

  title_.o = 0;
  figures_.clear();
  next_top_ = 0;
}

}  // namespace analysis
}  // namespace frc971
