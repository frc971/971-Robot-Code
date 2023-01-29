#include "Halide.h"

#define CHECK(x, message, ...)                                              \
  do {                                                                      \
    if (!(x)) {                                                             \
      fprintf(stderr, "assertion failed: " message ": %s\n", ##__VA_ARGS__, \
              #x);                                                          \
      abort();                                                              \
    }                                                                       \
  } while (0)

// This is a Halide "generator". This means it is a binary which generates
// ahead-of-time optimized functions as directed by command-line arguments.
// https://halide-lang.org/tutorials/tutorial_lesson_15_generators.html has an
// introduction to much of the magic in this file.

namespace frc971 {
namespace vision {
namespace {

// Returns a function implementating a 1-dimensional gaussian blur convolution.
Halide::Func GenerateBlur(std::string name, Halide::Func in, int col_step,
                          int row_step, int radius, std::vector<float> kernel,
                          Halide::Var col, Halide::Var row) {
  Halide::Expr expr = kernel[0] * in(col, row);
  for (int i = 1; i <= radius; ++i) {
    expr += kernel[0] * (in(col - i * col_step, row - i * row_step) +
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

class DecimateGenerator : public Halide::Generator<DecimateGenerator> {
 public:
  GeneratorParam<int> cols{"cols", 0};
  GeneratorParam<int> rows{"rows", 0};

  Input<Buffer<uint8_t>> input{"input", 3};
  Output<Buffer<uint8_t>> output{"output", 2};
  Output<Buffer<uint8_t>> decimated_output{"decimated_output", 2};

  Var col{"col"}, row{"row"};

  void generate() {
    CHECK(cols > 0, "Must specify a cols");
    CHECK(rows > 0, "Must specify a rows");

    input.dim(0).set_stride(2);
    input.dim(0).set_extent(cols);
    input.dim(0).set_min(0);

    input.dim(1).set_stride(cols * 2);
    input.dim(1).set_extent(rows);
    input.dim(1).set_min(0);

    input.dim(2).set_stride(1);
    input.dim(2).set_extent(2);
    input.dim(2).set_min(0);

    output(col, row) = input(col, row, 0);
    decimated_output(col, row) = output(col * 2, row * 2);

    decimated_output.compute_at(output, col);

    decimated_output.vectorize(col, 16);

    SetRowMajor(&output, cols, rows);

    SetRowMajor(&decimated_output, cols / 2, rows / 2);
  }
};

class ThresholdGenerator : public Halide::Generator<ThresholdGenerator> {
 public:
  GeneratorParam<int> rows{"rows", 0};
  GeneratorParam<int> cols{"cols", 0};

  Input<Buffer<uint8_t>> input{"input", 2};
  Output<Buffer<uint8_t>> output{"output", 2};

  Var x{"x"}, y{"y"};

  Func threshold{"threshold"}, threshold_max{"threshold_max"},
      threshold_min{"threshold_min"},
      convoluted_threshold_max{"convoluted_threshold_max"},
      convoluted_threshold_min{"convoluted_threshold_min"};

  void generate() {
    CHECK(cols > 0, "Columns must be more than 0");
    CHECK(rows > 0, "Rows must be more than 0");

    const int tile_size = 4;

    RDom r(0, tile_size, 0, tile_size);

    threshold_max(x, y) =
        maximum(input(r.x + x * tile_size, r.y + y * tile_size));
    threshold_min(x, y) =
        minimum(input(r.x + x * tile_size, r.y + y * tile_size));

    RDom r_conv(-1, 3, -1, 3);

    convoluted_threshold_max(x, y) =
        maximum(threshold_max(clamp(x + r_conv.x, 0, cols / tile_size - 1),
                              clamp(y + r_conv.y, 0, rows / tile_size - 1)));

    convoluted_threshold_min(x, y) =
        minimum(threshold_min(clamp(x + r_conv.x, 0, cols / tile_size - 1),
                              clamp(y + r_conv.y, 0, rows / tile_size - 1)));

    threshold(x, y) =
        convoluted_threshold_min(x, y) +
        (convoluted_threshold_max(x, y) - convoluted_threshold_min(x, y)) / 2;

    output(x, y) =
        select(convoluted_threshold_max(x / tile_size, y / tile_size) -
                       convoluted_threshold_min(x / tile_size, y / tile_size) <
                   5,
               Expr((uint8_t)(127)),
               select(input(x, y) > threshold(x / tile_size, y / tile_size),
                      Expr((uint8_t)(255)), Expr((uint8_t)(0))));

    SetRowMajor(&output, cols, rows);

    Var xi, yi;

    output.compute_root().tile(x, y, xi, yi, tile_size, tile_size);
    threshold.compute_root();
    convoluted_threshold_min.compute_root();
    convoluted_threshold_max.compute_root();
    threshold_min.compute_root();
    threshold_max.compute_root();
  }
};

}  // namespace vision
}  // namespace frc971

// TODO(austin): Combine the functions and optimize for device/host and all that
// jazz.
HALIDE_REGISTER_GENERATOR(frc971::vision::DecimateGenerator, decimate_generator)
HALIDE_REGISTER_GENERATOR(frc971::vision::ThresholdGenerator,
                          threshold_generator)
