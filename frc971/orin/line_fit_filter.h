#ifndef FRC971_ORIN_LINE_FIT_FILTER_H_
#define FRC971_ORIN_LINE_FIT_FILTER_H_

#include <cub/iterator/transform_input_iterator.cuh>
#include <cuda/std/tuple>

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "frc971/orin/cuda.h"

namespace frc971::apriltag {

// Class to hold the extents of a blob of points.
struct MinMaxExtents {
  // Min and max coordinates (in non-decimated coordinates)
  uint16_t min_x;
  uint16_t min_y;
  uint16_t max_x;
  uint16_t max_y;

  // The starting offset of this blob of points in the vector holding all the
  // points.
  uint32_t starting_offset;

  // The number of points in the blob.
  uint32_t count;

  // The dot product is:
  //  dot = sum( (px - cx) * gx) + (py - cy) * gy )
  //
  // We can split this up into:
  //  dot = sum(px * gx + py * gy) - cx * sum(gx) - cy * sum(gy)
  //
  // Which can be calculated without knowing the center.
  //
  // Since p and g are all integers, we can sum them into integers too so we
  // don't lose precision.
  int32_t gx_sum;
  int32_t gy_sum;

  int64_t pxgx_plus_pygy_sum;

  // Center location of the blob using the aprilrobotics algorithm.
  __host__ __device__ float cx() const {
    return (min_x + max_x) * 0.5f + 0.05118f;
  }
  __host__ __device__ float cy() const {
    return (min_y + max_y) * 0.5f + -0.028581f;
  }

  __host__ __device__ float dot() const {
    return static_cast<float>(pxgx_plus_pygy_sum * 2 -
                               (min_x + max_x) * gx_sum -
                               (min_y + max_y) * gy_sum) *
               0.5f -
           0.05118f * static_cast<float>(gx_sum) +
           0.028581f * static_cast<float>(gy_sum);
  }
};

__align__(16) struct LineFitPoint {
  // TODO(austin): How much precision do we actually need?  The less, the
  // faster...  The less memory too, the faster.
  //
  // Is it the double's or the memory which makes us slow?  Could probably bit
  // pack ints in here...
  int64_t Mxx;
  int64_t Myy;
  int64_t Mxy;
  // TODO(austin): These both fit in 4 byte numbers :)
  // There are at most 2 * (width + height) -> 13 bits points in a blob.
  // The weight can be at most 8 bits (probably 9 because it has a sign).
  // A point has 11 bits of position data in it.
  //
  // To do this with sub bit precision:
  //   2 * (1088 + 1456) * 1456 * 255 -> 0x7098f600
  //
  // Which is < 31 bits, so we can hold a sign bit too!
  int32_t Mx;
  int32_t My;
  int32_t W;
  uint32_t blob_index;
};

struct LineFitMoments {
  // See LineFitPoint for more info.
  int32_t Mx;
  int32_t My;
  int32_t W;
  int64_t Mxx;
  int64_t Myy;
  int64_t Mxy;
  int N;  // how many points are included in the set?
};

std::ostream &operator<<(std::ostream &os,
                         const frc971::apriltag::LineFitMoments &moments);

struct Peak {
  static constexpr uint16_t kNoPeak() { return 0xffff; }
  float error;
  // Point index.
  uint32_t filtered_point_index;
  // 0xffff if this isn't a peak, otherwise the blob index.
  uint16_t blob_index;
};

struct PeakExtents {
  uint16_t blob_index;
  uint32_t starting_offset;
  uint32_t count;
};

struct PeakDecomposer {
  static constexpr size_t kBitsInKey = 16 + 32;
  __host__ __device__ ::cuda::std::tuple<uint16_t &, float &> operator()(
      Peak &key) const {
    return {key.blob_index, key.error};
  }
};

constexpr std::array<float, 7> FilterCoefficients() {
  return std::array<float, 7>{
      0.01110899634659290314, 0.13533528149127960205, 0.60653066635131835938,
      1.00000000000000000000, 0.60653066635131835938, 0.13533528149127960205,
      0.01110899634659290314,
  };
}

struct FitQuad {
  uint16_t blob_index;
  bool valid;
  uint16_t indices[4];
  LineFitMoments moments[4];
};

__host__ __device__ void FitLine(LineFitMoments moments, double *lineparam01,
                                 double *lineparam23, double *err, double *mse);

void FitLines(
    const LineFitPoint *line_fit_points_device, size_t points,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    size_t num_extents, double *errs_device, double *filtered_errs_device,
    Peak *peaks_device, CudaStream *stream);

void FitQuads(
    const Peak *peaks_device, size_t peaks, const PeakExtents *peak_extents,
    size_t num_extents, const LineFitPoint *line_fit_points_device, int nmaxima,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    float max_line_fit_mse, double cos_critical_rad, FitQuad *fit_quads_device,
    CudaStream *stream);

// Returns the m0, m1, m2, m3 indices for the provided index.  The index is the
// inner loop number when you process the 4 for loops in order and count.  See
// FilterTest.Unrank for an example.
//
// This lets us distribute work amoung the cuda threads and get back the index.
__host__ __device__ std::tuple<uint, uint, uint, uint> Unrank(uint i);
// The max number of work elements for a max maxes of 10.
constexpr size_t MaxRankedIndex() { return 210; }

}  // namespace frc971::apriltag

#endif  // FRC971_ORIN_LINE_FIT_FILTER_H_
