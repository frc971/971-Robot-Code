#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include "Halide.h"
#include "glog/logging.h"

// This is a Halide "generator". This means it is a binary which generates
// ahead-of-time optimized functions as directed by command-line arguments.
// https://halide-lang.org/tutorials/tutorial_lesson_15_generators.html has an
// introduction to much of the magic in this file.

namespace frc971 {
namespace vision {
namespace {

// Returns a function implementating a 1-dimensional gaussian blur convolution.
Halide::Func GenerateBlur(std::string name, Halide::Func in, int col_step,
                          int row_step, int radius, cv::Mat kernel,
                          Halide::Var col, Halide::Var row) {
  Halide::Expr expr = kernel.at<float>(0) * in(col, row);
  for (int i = 1; i <= radius; ++i) {
    expr += kernel.at<float>(i) * (in(col - i * col_step, row - i * row_step) +
                                   in(col + i * col_step, row + i * row_step));
  }
  Halide::Func func(name);
  func(col, row) = expr;
  return func;
}

template <typename T>
void SetRowMajor(T *buffer_parameter, int cols, int rows) {
  buffer_parameter->dim(0).set_stride(1);
  buffer_parameter->dim(0).set_extent(cols);
  buffer_parameter->dim(0).set_min(0);
  buffer_parameter->dim(1).set_stride(cols);
  buffer_parameter->dim(1).set_extent(rows);
  buffer_parameter->dim(1).set_min(0);
}

}  // namespace

class GaussianGenerator : public Halide::Generator<GaussianGenerator> {
 public:
  GeneratorParam<int> cols{"cols", 0};
  GeneratorParam<int> rows{"rows", 0};
  GeneratorParam<double> sigma{"sigma", -1};
  GeneratorParam<int> filter_width{"filter_width", 0};

  Input<Buffer<int16_t>> input{"input", 2};
  Output<Buffer<int16_t>> output{"output", 2};

  // We use opencv's naming convention, instead of the (x, y) which most of the
  // halide examples use. This is easier to keep straight with the row-major
  // storage order we're using though.
  // col is first because incrementing the data index by 1 moves over 1 column.
  Var col{"col"}, row{"row"};

  void generate() {
    CHECK(cols > 0) << ": Must specify a cols";
    CHECK(rows > 0) << ": Must specify a rows";
    CHECK(sigma > 0) << ": Must specify a sigma";
    CHECK(filter_width > 0) << ": Must specify a filter_width";
    CHECK((filter_width % 2) == 1)
        << ": Invalid filter_width: " << static_cast<int>(filter_width);

    SetRowMajor(&input, cols, rows);

    const int radius = (filter_width - 1) / 2;
    const cv::Mat kernel =
        cv::getGaussianKernel(filter_width, sigma, CV_32F)
            .rowRange(radius, filter_width);

    Halide::Func in_bounded = Halide::BoundaryConditions::repeat_edge(input);
    Halide::Func blur_col =
        GenerateBlur("blur_col", in_bounded, 1, 0, radius, kernel, col, row);
    output(col, row) = Halide::cast<int16_t>(
        GenerateBlur("blur_row", blur_col, 0, 1, radius, kernel, col, row)(col, row));

    // Vectorize along the col dimension. Most of the data needed by each lane
    // overlaps this way. This also has the advantage of being the first
    // dimension, so as we move along it we will have good data locality.
    blur_col.vectorize(col, 8);

    // The fun part: we tile the algorithm. This tile size is designed to fit
    // within each CPU core's L1 cache. On the boundaries of the tiles, we end
    // re-computing the first blur, but fitting within the L1 cache is worth it.
    Halide::Var col_inner("col_inner"), row_inner("row_inner");
    output.tile(col, row, col_inner, row_inner, 64, 32);
    Halide::Var tile_index("tile_index");
    output.fuse(col, row, tile_index);

    // Compute the first blur as needed for the second one, within each tile.
    blur_col.compute_at(output, tile_index);
    // And then vectorize the second blur within each tile.
    output.vectorize(col_inner, 8);

    // Lastly, compute all the tiles in parallel.
    output.parallel(tile_index);

    SetRowMajor(&output, cols, rows);
  }
};

class SubtractGenerator : public Halide::Generator<SubtractGenerator> {
 public:
  GeneratorParam<int> cols{"cols", 0};
  GeneratorParam<int> rows{"rows", 0};

  Input<Buffer<int16_t>> input_a{"input_a", 2};
  Input<Buffer<int16_t>> input_b{"input_b", 2};
  Output<Buffer<int16_t>> output{"output", 2};

  Var col{"col"}, row{"row"};

  void generate() {
    CHECK(cols > 0) << ": Must specify a cols";
    CHECK(rows > 0) << ": Must specify a rows";

    SetRowMajor(&input_a, cols, rows);
    SetRowMajor(&input_b, cols, rows);

    output(col, row) = Halide::saturating_cast<int16_t>(
        Halide::cast<int32_t>(input_a(col, row)) - input_b(col, row));
    output.vectorize(col, 16);

    SetRowMajor(&output, cols, rows);
  }
};

class GaussianAndSubtractGenerator
    : public Halide::Generator<GaussianAndSubtractGenerator> {
 public:
  GeneratorParam<int> cols{"cols", 0};
  GeneratorParam<int> rows{"rows", 0};
  GeneratorParam<double> sigma{"sigma", -1};
  GeneratorParam<int> filter_width{"filter_width", 0};

  Input<Buffer<int16_t>> input{"input", 2};
  Output<Buffer<int16_t>> blurred{"blurred", 2};
  Output<Buffer<int16_t>> difference{"difference", 2};

  // We use opencv's naming convention, instead of the (x, y) which most of the
  // halide examples use. This is easier to keep straight with the row-major
  // storage order we're using though.
  // col is first because incrementing the data index by 1 moves over 1 column.
  Var col{"col"}, row{"row"};

  void generate() {
    CHECK(cols > 0) << ": Must specify a cols";
    CHECK(rows > 0) << ": Must specify a rows";
    CHECK(sigma > 0) << ": Must specify a sigma";
    CHECK(filter_width > 0) << ": Must specify a filter_width";
    CHECK((filter_width % 2) == 1)
        << ": Invalid filter_width: " << static_cast<int>(filter_width);

    SetRowMajor(&input, cols, rows);

    const int radius = (filter_width - 1) / 2;
    const cv::Mat kernel =
        cv::getGaussianKernel(filter_width, sigma, CV_32F)
            .rowRange(radius, filter_width);

    Halide::Func in_bounded = Halide::BoundaryConditions::repeat_edge(input);
    Halide::Func blur_col =
        GenerateBlur("blur_col", in_bounded, 1, 0, radius, kernel, col, row);
    blurred(col, row) = Halide::cast<int16_t>(
        GenerateBlur("blur_row", blur_col, 0, 1, radius, kernel, col, row)(col, row));
    difference(col, row) = Halide::saturating_cast<int16_t>(
        Halide::cast<int32_t>(blurred(col, row)) - input(col, row));

    // Vectorize along the col dimension. Most of the data needed by each lane
    // overlaps this way. This also has the advantage of being the first
    // dimension, so as we move along it we will have good data locality.
    blur_col.vectorize(col, 8);

    // The fun part: we tile the algorithm. This tile size is designed to fit
    // within each CPU core's L1 cache. On the boundaries of the tiles, we end
    // re-computing the first blur, but fitting within the L1 cache is worth it.
    Halide::Var col_inner("col_inner"), row_inner("row_inner");
    blurred.tile(col, row, col_inner, row_inner, 64, 32);
    Halide::Var tile_index("tile_index");
    blurred.fuse(col, row, tile_index);

    // Compute the first blur as needed for the second one, within each tile.
    blur_col.compute_at(blurred, tile_index);
    // And then vectorize the second blur within each tile.
    blurred.vectorize(col_inner, 8);

    // Lastly, compute all the tiles in parallel.
    blurred.parallel(tile_index);
    blurred.compute_root();

    // TODO(Brian): Calulate difference within each of the tiles to speed things
    // up.

    SetRowMajor(&blurred, cols, rows);
    SetRowMajor(&difference, cols, rows);
  }
};

}  // namespace vision
}  // namespace frc971

HALIDE_REGISTER_GENERATOR(frc971::vision::GaussianGenerator, gaussian_generator)
HALIDE_REGISTER_GENERATOR(frc971::vision::SubtractGenerator, subtract_generator)
HALIDE_REGISTER_GENERATOR(frc971::vision::GaussianAndSubtractGenerator,
                          gaussian_and_subtract_generator)
