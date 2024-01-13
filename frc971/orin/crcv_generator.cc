#include <iostream>

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

namespace frc971::orin {
namespace {

template <typename T>
void SetRowMajor(T *buffer_parameter, int cols, int rows, int channels) {
  buffer_parameter->dim(0).set_stride(channels);
  buffer_parameter->dim(0).set_extent(cols);
  buffer_parameter->dim(0).set_min(0);

  buffer_parameter->dim(1).set_stride(cols * channels);
  buffer_parameter->dim(1).set_extent(rows);
  buffer_parameter->dim(1).set_min(0);

  buffer_parameter->dim(2).set_stride(1);
  buffer_parameter->dim(2).set_extent(channels);
  buffer_parameter->dim(2).set_min(0);
}
}  // namespace

// Takes an image with y in one plane with a provided stride, and cbcr in
// another with a provided stride and makes a ycbcr output image.
class YCbCr : public Halide::Generator<YCbCr> {
 public:
  GeneratorParam<int> cols{"cols", 0};
  GeneratorParam<int> rows{"rows", 0};
  GeneratorParam<int> ystride{"ystride", 0};
  GeneratorParam<int> cbcrstride{"cbcrstride", 0};

  Input<Buffer<uint8_t, 2>> input_y{"y"};
  Input<Buffer<uint8_t, 3>> input_cbcr{"cbcr"};
  Output<Buffer<uint8_t, 3>> output{"output"};

  Var col{"col"}, row{"row"}, channel{"channel"};

  // Everything is indexed as col, row, channel.
  void generate() {
    CHECK(cols > 0, "Must specify a cols");
    CHECK(rows > 0, "Must specify a rows");

    input_y.dim(0).set_stride(1);
    input_y.dim(0).set_extent(cols);
    input_y.dim(0).set_min(0);

    input_y.dim(1).set_stride(ystride);
    input_y.dim(1).set_extent(rows);
    input_y.dim(1).set_min(0);

    input_cbcr.dim(0).set_stride(2);
    input_cbcr.dim(0).set_extent(cols);
    input_cbcr.dim(0).set_min(0);

    input_cbcr.dim(1).set_stride(cbcrstride);
    input_cbcr.dim(1).set_extent(rows);
    input_cbcr.dim(1).set_min(0);

    input_cbcr.dim(2).set_stride(1);
    input_cbcr.dim(2).set_extent(2);
    input_cbcr.dim(2).set_min(0);

    output(col, row, channel) =
        Halide::select(channel == 0, input_y(col, row),
                       Halide::select(channel == 1, input_cbcr(col, row, 0),
                                      input_cbcr(col, row, 1)));

    output.reorder(channel, col, row);
    output.unroll(channel);

    output.vectorize(col, 8);
    output.unroll(col, 4);

    SetRowMajor(&output, cols, rows, 3);
  }
};

class YCbCr422 : public Halide::Generator<YCbCr422> {
 public:
  GeneratorParam<int> cols{"cols", 0};
  GeneratorParam<int> rows{"rows", 0};
  GeneratorParam<int> ystride{"ystride", 0};
  GeneratorParam<int> cbcrstride{"cbcrstride", 0};

  Input<Buffer<uint8_t, 2>> input_y{"y"};
  Input<Buffer<uint8_t, 3>> input_cbcr{"cbcr"};
  Output<Buffer<uint8_t, 3>> output{"output"};

  Var col{"col"}, row{"row"}, channel{"channel"};

  // Everything is indexed as col, row, channel.
  void generate() {
    CHECK(cols > 0, "Must specify a cols");
    CHECK((cols % 2) == 0, "Must specify a cols with an even number of cols");
    CHECK(rows > 0, "Must specify a rows");

    input_y.dim(0).set_stride(1);
    input_y.dim(0).set_extent(cols);
    input_y.dim(0).set_min(0);

    input_y.dim(1).set_stride(ystride);
    input_y.dim(1).set_extent(rows);
    input_y.dim(1).set_min(0);

    input_cbcr.dim(0).set_stride(2);
    input_cbcr.dim(0).set_extent(cols / 2);
    input_cbcr.dim(0).set_min(0);

    input_cbcr.dim(1).set_stride(cbcrstride);
    input_cbcr.dim(1).set_extent(rows);
    input_cbcr.dim(1).set_min(0);

    input_cbcr.dim(2).set_stride(1);
    input_cbcr.dim(2).set_extent(2);
    input_cbcr.dim(2).set_min(0);

    output(col, row, channel) = Halide::select(
        channel == 0, input_y(col, row), input_cbcr(col / 2, row, col % 2));

    output.reorder(channel, col, row);
    output.unroll(channel);

    output.vectorize(col, 8);
    output.unroll(col, 4);

    SetRowMajor(&output, cols, rows, 2);
  }
};

}  // namespace frc971::orin

HALIDE_REGISTER_GENERATOR(frc971::orin::YCbCr, ycbcr)
HALIDE_REGISTER_GENERATOR(frc971::orin::YCbCr422, ycbcr422)
