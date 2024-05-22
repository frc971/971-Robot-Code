#include "frc971/orin/apriltag.h"

#include <thrust/iterator/constant_iterator.h>
#include <thrust/iterator/transform_iterator.h>

#include <cub/device/device_copy.cuh>
#include <cub/device/device_radix_sort.cuh>
#include <cub/device/device_reduce.cuh>
#include <cub/device/device_run_length_encode.cuh>
#include <cub/device/device_scan.cuh>
#include <cub/device/device_segmented_sort.cuh>
#include <cub/device/device_select.cuh>
#include <cub/iterator/discard_output_iterator.cuh>
#include <cub/iterator/transform_input_iterator.cuh>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "third_party/apriltag/common/g2d.h"

#include "aos/time/time.h"
#include "frc971/orin/labeling_allegretti_2019_BKE.h"
#include "frc971/orin/threshold.h"
#include "frc971/orin/transform_output_iterator.h"

namespace frc971::apriltag {
namespace {

typedef std::chrono::duration<float, std::milli> float_milli;

// Returns true if the QuadBoundaryPoint is nonzero.
struct NonZero {
  __host__ __device__ __forceinline__ bool operator()(
      const QuadBoundaryPoint &a) const {
    return a.nonzero();
  }
};

// Always returns true (only used for scratch space calcs).
template <typename T>
struct True {
  __host__ __device__ __forceinline__ bool operator()(const T &) const {
    return true;
  }
};

// Computes and returns the scratch space needed for DeviceRadixSort::SortKeys
// of the provided key with the provided number of elements.
template <typename T>
static size_t RadixSortScratchSpace(size_t elements) {
  size_t temp_storage_bytes = 0;
  QuadBoundaryPointDecomposer decomposer;
  cub::DeviceRadixSort::SortKeys(nullptr, temp_storage_bytes, (T *)(nullptr),
                                 (T *)(nullptr), elements, decomposer);
  return temp_storage_bytes;
}

// Computes and returns the scratch space needed for DeviceSelect::If of the
// provided type Tin, to be written to the provided type Tout, for the provided
// number of elements.  num_markers is the device pointer used to hold the
// selected number of elements.
template <typename Tin, typename Tout>
static size_t DeviceSelectIfScratchSpace(size_t elements, int *num_markers) {
  size_t temp_storage_bytes = 0;
  CHECK_CUDA(cub::DeviceSelect::If(nullptr, temp_storage_bytes,
                                   (Tin *)(nullptr), (Tout *)(nullptr),
                                   num_markers, elements, True<Tin>()));
  return temp_storage_bytes;
}

// Always returns the first element (only used for scratch space calcs).
template <typename T>
struct CustomFirst {
  __host__ __device__ __forceinline__ T operator()(const T &a,
                                                   const T & /*b*/) const {
    return a;
  }
};

// Computes and returns the scratch space needed for DeviceReduce::ReduceByKey
// of the provided key K and value V with the provided number of elements.
template <typename K, typename V>
static size_t DeviceReduceByKeyScratchSpace(size_t elements) {
  size_t temp_storage_bytes = 0;
  CHECK_CUDA(cub::DeviceReduce::ReduceByKey(
      nullptr, temp_storage_bytes, (K *)(nullptr), (K *)(nullptr),
      (V *)(nullptr), (V *)(nullptr), (size_t *)(nullptr), CustomFirst<V>(),
      elements));
  return temp_storage_bytes;
}

// Computes and returns the scratch space needed for DeviceScan::InclusiveScan
// of the provided value V with the provided number of elements.
template <typename V>
static size_t DeviceScanInclusiveScanScratchSpace(size_t elements) {
  size_t temp_storage_bytes = 0;
  CHECK_CUDA(cub::DeviceScan::InclusiveScan(nullptr, temp_storage_bytes,
                                            (V *)(nullptr), (V *)(nullptr),
                                            CustomFirst<V>(), elements));
  return temp_storage_bytes;
}

// Computes and returns the scratch space needed for DeviceScan::InclusiveScan
// of the provided value V with the provided number of elements.
template <typename K, typename V>
static size_t DeviceScanInclusiveScanByKeyScratchSpace(size_t elements) {
  size_t temp_storage_bytes = 0;
  CHECK_CUDA(cub::DeviceScan::InclusiveScanByKey(
      nullptr, temp_storage_bytes, (K *)(nullptr), (V *)(nullptr),
      (V *)(nullptr), CustomFirst<V>(), elements));
  return temp_storage_bytes;
}

}  // namespace

template <InputFormat INPUT_FORMAT>
constexpr size_t InputFormatToChannels(void)
{
  if constexpr (INPUT_FORMAT == InputFormat::Mono8) {
    return 1;
  }
  if constexpr (INPUT_FORMAT == InputFormat::YCbCr422) {
    return 2;
  }
  if constexpr (INPUT_FORMAT == InputFormat::BGR8) {
    return 3;
  }
  // TODO : Probably need a throw or assert here
}

template<size_t MULTIPLE> size_t roundUp(const size_t value)
{
  return ((value + MULTIPLE - 1) / MULTIPLE) * MULTIPLE;
}

template <InputFormat INPUT_FORMAT>
GpuDetector<INPUT_FORMAT>::GpuDetector(size_t width, size_t height,
                                       apriltag_detector_t *tag_detector,
                                       CameraMatrix camera_matrix,
                                       DistCoeffs distortion_coefficients)
    : width_(width),
      height_(roundUp<8>(height)),
      // Save actual input size. Round up everything else to have a height
      // that's a multiple of 8.  The initial memcpy into the GPU color image
      // will just copy the actual number of bytes in the input image. This
      // could leave a few rows in the GPU color image uninitialized, but that
      // is handled in the GpuMemory constructor which memsets the whole
      // block to 0.  From there the extra rows are never written.
      // From that point, hopefully the rest of the code thinks the image is
      // the rounded up size, so the algorithm will work on any size image.
      input_size_(width * height * InputFormatToChannels<INPUT_FORMAT>()),
      tag_detector_(tag_detector),
      gray_image_host_(width * height_),
      color_image_device_(width * height_ * InputFormatToChannels<INPUT_FORMAT>()),
      gray_image_device_(width * height_),
      decimated_image_device_(width / 2 * height_ / 2),
      unfiltered_minmax_image_device_((width / 2 / 4 * height_ / 2 / 4) * 2),
      minmax_image_device_((width / 2 / 4 * height_ / 2 / 4) * 2),
      thresholded_image_device_(width / 2 * height_ / 2),
      union_markers_device_(width / 2 * height_ / 2),
      union_markers_size_device_(width / 2 * height_ / 2),
      union_marker_pair_device_((width / 2 - 2) * (height_ / 2 - 2) * 4),
      compressed_union_marker_pair_device_(union_marker_pair_device_.size()),
      sorted_union_marker_pair_device_(union_marker_pair_device_.size()),
      extents_device_(union_marker_pair_device_.size()),
      selected_extents_device_(kMaxBlobs),
      selected_blobs_device_(union_marker_pair_device_.size()),
      sorted_selected_blobs_device_(selected_blobs_device_.size()),
      line_fit_points_device_(selected_blobs_device_.size()),
      errs_device_(line_fit_points_device_.size()),
      filtered_errs_device_(line_fit_points_device_.size()),
      filtered_is_local_peak_device_(line_fit_points_device_.size()),
      compressed_peaks_device_(line_fit_points_device_.size()),
      sorted_compressed_peaks_device_(line_fit_points_device_.size()),
      peak_extents_device_(kMaxBlobs),
      camera_matrix_(camera_matrix),
      distortion_coefficients_(distortion_coefficients),
      fit_quads_device_(kMaxBlobs),
      radix_sort_tmpstorage_device_(RadixSortScratchSpace<QuadBoundaryPoint>(
          sorted_union_marker_pair_device_.size())),
      temp_storage_compressed_union_marker_pair_device_(
          DeviceSelectIfScratchSpace<QuadBoundaryPoint, QuadBoundaryPoint>(
              union_marker_pair_device_.size(),
              num_compressed_union_marker_pair_device_.get())),
      temp_storage_bounds_reduce_by_key_device_(
          DeviceReduceByKeyScratchSpace<uint64_t, MinMaxExtents>(
              union_marker_pair_device_.size())),
      temp_storage_dot_product_device_(
          DeviceReduceByKeyScratchSpace<uint64_t, float>(
              union_marker_pair_device_.size())),
      temp_storage_compressed_filtered_blobs_device_(
          DeviceSelectIfScratchSpace<IndexPoint, IndexPoint>(
              union_marker_pair_device_.size(),
              num_selected_blobs_device_.get())),
      temp_storage_selected_extents_scan_device_(
          DeviceScanInclusiveScanScratchSpace<
              cub::KeyValuePair<long, MinMaxExtents>>(kMaxBlobs)),
      temp_storage_line_fit_scan_device_(
          DeviceScanInclusiveScanByKeyScratchSpace<uint32_t, LineFitPoint>(
              sorted_selected_blobs_device_.size())) {
  // If the input image is grayscale, alias the gray image to the color image.
  // InternalCudaToGreyscaleAndDecimateHalide will skip copying between the
  // color and gray images as a result.
  if constexpr (INPUT_FORMAT == InputFormat::Mono8) {
    gray_image_device_ = color_image_device_;
  }
  fit_quads_host_.reserve(kMaxBlobs);
  quad_corners_host_.reserve(kMaxBlobs);

  CHECK_EQ(tag_detector_->quad_decimate, 2);
  CHECK(!tag_detector_->qtp.deglitch);

  for (int i = 0; i < zarray_size(tag_detector_->tag_families); i++) {
    apriltag_family_t *family;
    zarray_get(tag_detector_->tag_families, i, &family);
    if (family->width_at_border < min_tag_width_) {
      min_tag_width_ = family->width_at_border;
    }
    normal_border_ |= !family->reversed_border;
    reversed_border_ |= family->reversed_border;
  }
  min_tag_width_ /= tag_detector_->quad_decimate;
  if (min_tag_width_ < 3) {
    min_tag_width_ = 3;
  }

  poly0_ = g2d_polygon_create_zeros(4);
  poly1_ = g2d_polygon_create_zeros(4);

  detections_ = zarray_create(sizeof(apriltag_detection_t *));
  zarray_ensure_capacity(detections_, kMaxBlobs);
}

template <InputFormat INPUT_FORMAT>
GpuDetector<INPUT_FORMAT>::~GpuDetector() {
  for (int i = 0; i < zarray_size(detections_); ++i) {
    apriltag_detection_t *det;
    zarray_get(detections_, i, &det);
    apriltag_detection_destroy(det);
  }

  zarray_destroy(detections_);
  zarray_destroy(poly1_);
  zarray_destroy(poly0_);
}

namespace {

// Computes a massive image of 4x QuadBoundaryPoint per pixel with a
// QuadBoundaryPoint for each pixel pair which crosses a blob boundary.
template <size_t kBlockWidth, size_t kBlockHeight>
__global__ void BlobDiff(const uint8_t *thresholded_image,
                         const uint32_t *blobs,
                         const uint32_t *union_markers_size,
                         QuadBoundaryPoint *result, size_t width,
                         size_t height) {
  __shared__ uint32_t temp_blob_storage[kBlockWidth * kBlockHeight];
  __shared__ uint8_t temp_image_storage[kBlockWidth * kBlockHeight];

  // We overlap both directions in X, and only one direction in Y.
  const uint x = blockIdx.x * (blockDim.x - 2) + threadIdx.x;
  const uint y = blockIdx.y * (blockDim.y - 1) + threadIdx.y;

  // Ignore anything outside the image.
  if (x >= width || y >= height || y == 0) {
    return;
  }

  // Compute the location in the image this threads is responsible for.
  const uint global_input_index = x + y * width;
  // And the destination in the temporary storage to save it.
  const uint thread_linear_index = threadIdx.x + blockDim.x * threadIdx.y;

  // Now, load all the data.
  const uint32_t rep0 = temp_blob_storage[thread_linear_index] =
      blobs[global_input_index];
  const uint8_t v0 = temp_image_storage[thread_linear_index] =
      thresholded_image[global_input_index];

  // All threads in the block have now gotten this far so the shared memory is
  // consistent.
  __syncthreads();

  // We've done our job loading things into memory, this pixel is a boundary
  // with the upper and left sides, or covered by the next block.
  if (threadIdx.x == 0) {
    return;
  }

  // We need to search both ways in x, this thread doesn't participate further
  // :(
  if (threadIdx.x == blockDim.x - 1) {
    return;
  }

  if (threadIdx.y == blockDim.y - 1) {
    return;
  }

  // This is the last pixel along the lower and right sides, we don't need to
  // compute it.
  if (x == width - 1 || y == height - 1) {
    return;
  }

  // Place in memory to write the result.
  const uint global_output_index = (x - 1) + (y - 1) * (width - 2);

  // Short circuit 127's and write an empty point out.
  if (v0 == 127 || union_markers_size[rep0] < 25) {
#pragma unroll
    for (size_t point_offset = 0; point_offset < 4; ++point_offset) {
      const size_t write_address =
          (width - 2) * (height - 2) * point_offset + global_output_index;
      result[write_address] = QuadBoundaryPoint();
    }
    return;
  }

  uint32_t rep1;
  uint8_t v1;

#define DO_CONN(dx, dy, point_offset)                                    \
  {                                                                      \
    QuadBoundaryPoint cluster_id;                                        \
    const uint x1 = dx + threadIdx.x;                                    \
    const uint y1 = dy + threadIdx.y;                                    \
    const uint thread_linear_index1 = x1 + blockDim.x * y1;              \
    v1 = temp_image_storage[thread_linear_index1];                       \
    rep1 = temp_blob_storage[thread_linear_index1];                      \
    if (v0 + v1 == 255) {                                                \
      if (union_markers_size[rep1] >= 25) {                              \
        if (rep0 < rep1) {                                               \
          cluster_id.set_rep1(rep1);                                     \
          cluster_id.set_rep0(rep0);                                     \
        } else {                                                         \
          cluster_id.set_rep1(rep0);                                     \
          cluster_id.set_rep0(rep1);                                     \
        }                                                                \
        cluster_id.set_base_xy(x, y);                                    \
        cluster_id.set_dxy(point_offset);                                \
        cluster_id.set_black_to_white(v1 > v0);                          \
      }                                                                  \
    }                                                                    \
    const size_t write_address =                                         \
        (width - 2) * (height - 2) * point_offset + global_output_index; \
    result[write_address] = cluster_id;                                  \
  }

  // We search the following 4 neighbors.
  //      ________
  //      | x | 0 |
  //  -------------
  //  | 3 | 2 | 1 |
  //  -------------
  //
  //  If connection 3 has the same IDs as the connection between blocks 0 and 2,
  //  we will have a duplicate entry.  Detect and don't add it.  This will only
  //  happen if id(x) == id(0) and id(2) == id(3),
  //         or id(x) == id(2) and id(0) ==id(3).

  DO_CONN(1, 0, 0);
  DO_CONN(1, 1, 1);
  DO_CONN(0, 1, 2);
  const uint64_t rep_block_2 = rep1;
  const uint8_t v1_block_2 = v1;

  const uint left_thread_linear_index1 =
      threadIdx.x - 1 + blockDim.x * threadIdx.y;
  const uint8_t v1_block_left = temp_image_storage[left_thread_linear_index1];
  const uint32_t rep_block_left = temp_blob_storage[left_thread_linear_index1];

  // Do the dedup calculation now.
  if (v1_block_left != 127 && v1_block_2 != 127 &&
      v1_block_2 != v1_block_left) {
    if (x != 1 && union_markers_size[rep_block_left] >= 25 &&
        union_markers_size[rep_block_2] >= 25) {
      const size_t write_address =
          (width - 2) * (height - 2) * 3 + global_output_index;
      result[write_address] = QuadBoundaryPoint();
      return;
    }
  }

  DO_CONN(-1, 1, 3);
}

// Masks out just the blob ID pair, rep01.
struct MaskRep01 {
  __host__ __device__ __forceinline__ uint64_t
  operator()(const QuadBoundaryPoint &a) const {
    return a.rep01();
  }
};

// Masks out just the blob ID pair, rep01.
struct MaskBlobIndex {
  __host__ __device__ __forceinline__ uint32_t
  operator()(const IndexPoint &a) const {
    return a.blob_index();
  }
};

// Rewrites a QuadBoundaryPoint to an IndexPoint, adding the angle to the
// center.
class RewriteToIndexPoint {
 public:
  RewriteToIndexPoint(MinMaxExtents *extents_device, size_t num_extents)
      : blob_finder_(extents_device, num_extents) {}

  __host__ __device__ __forceinline__ IndexPoint
  operator()(cub::KeyValuePair<long, QuadBoundaryPoint> pt) const {
    size_t index = blob_finder_.FindBlobIndex(pt.key);
    IndexPoint result(index, pt.value.point_bits());
    return result;
  }

  BlobExtentsIndexFinder blob_finder_;
};

// Calculates Theta for a given IndexPoint
class AddThetaToIndexPoint {
 public:
  AddThetaToIndexPoint(MinMaxExtents *extents_device, size_t num_extents)
      : blob_finder_(extents_device, num_extents) {}
  __host__ __device__ __forceinline__ IndexPoint operator()(IndexPoint a) {
    MinMaxExtents extents = blob_finder_.Get(a.blob_index());
    float theta =
        (atan2f(a.y() - extents.cy(), a.x() - extents.cx()) + M_PI) * 8e6;
    long long int theta_int = llrintf(theta);

    a.set_theta(std::max<long long int>(0, theta_int));
    return a;
  }

 private:
  BlobExtentsIndexFinder blob_finder_;
};

// TODO(austin): Make something which rewrites points on the way back out to
// memory and adds the slope.

// Transforms aQuadBoundaryPoint into a single point extent for Reduce.
struct TransformQuadBoundaryPointToMinMaxExtents {
  __host__ __device__ __forceinline__ MinMaxExtents
  operator()(cub::KeyValuePair<long, QuadBoundaryPoint> pt) const {
    MinMaxExtents result;
    result.min_y = result.max_y = pt.value.y();
    result.min_x = result.max_x = pt.value.x();
    result.starting_offset = pt.key;
    result.count = 1;
    result.pxgx_plus_pygy_sum =
        static_cast<int64_t>(pt.value.x()) * pt.value.gx() +
        static_cast<int64_t>(pt.value.y()) * pt.value.gy();
    result.gx_sum = pt.value.gx();
    result.gy_sum = pt.value.gy();
    return result;
  }
};

// Reduces 2 extents by tracking the range and updating the offset and count
// accordingly.
struct QuadBoundaryPointExtents {
  __host__ __device__ __forceinline__ MinMaxExtents
  operator()(const MinMaxExtents &a, const MinMaxExtents &b) const {
    MinMaxExtents result;
    result.min_x = std::min(a.min_x, b.min_x);
    result.max_x = std::max(a.max_x, b.max_x);
    result.min_y = std::min(a.min_y, b.min_y);
    result.max_y = std::max(a.max_y, b.max_y);
    // And the actual start is the first of the points.
    result.starting_offset = std::min(a.starting_offset, b.starting_offset);
    // We want to count everything.
    result.count = a.count + b.count;
    result.pxgx_plus_pygy_sum = a.pxgx_plus_pygy_sum + b.pxgx_plus_pygy_sum;
    result.gx_sum = a.gx_sum + b.gx_sum;
    result.gy_sum = a.gy_sum + b.gy_sum;
    return result;
  }
};

class NonzeroBlobs {
 public:
  __host__ __device__
  NonzeroBlobs(const cub::KeyValuePair<long, MinMaxExtents> *extents_device)
      : extents_device_(extents_device) {}

  __host__ __device__ __forceinline__ bool operator()(
      const IndexPoint &a) const {
    return extents_device_[a.blob_index()].value.count > 0;
  }

 private:
  const cub::KeyValuePair<long, MinMaxExtents> *extents_device_;
};

// Selects blobs which are big enough, not too big, and have the right color in
// the middle.
class SelectBlobs {
 public:
  SelectBlobs(const MinMaxExtents *extents_device, size_t tag_width,
              bool reversed_border, bool normal_border,
              size_t min_cluster_pixels, size_t max_cluster_pixels)
      : extents_device_(extents_device),
        tag_width_(tag_width),
        reversed_border_(reversed_border),
        normal_border_(normal_border),
        min_cluster_pixels_(std::max<size_t>(24u, min_cluster_pixels)),
        max_cluster_pixels_(max_cluster_pixels) {}

  // Returns true if the blob passes the size and dot product checks and is
  // worth further consideration.
  __host__ __device__ __forceinline__ bool operator()(
      MinMaxExtents extents) const {
    if (extents.count < min_cluster_pixels_) {
      return false;
    }
    if (extents.count > max_cluster_pixels_) {
      return false;
    }

    // Area must also be reasonable.
    if ((extents.max_x - extents.min_x) * (extents.max_y - extents.min_y) <
        tag_width_) {
      return false;
    }

    // And the right side must be inside.
    const bool quad_reversed_border = extents.dot() < 0.0;
    if (!reversed_border_ && quad_reversed_border) {
      return false;
    }
    if (!normal_border_ && !quad_reversed_border) {
      return false;
    }

    return true;
  }

  __host__ __device__ __forceinline__ bool operator()(
      const IndexPoint &a) const {
    bool result = (*this)(extents_device_[a.blob_index()]);

    return result;
  }

  const MinMaxExtents *extents_device_;
  size_t tag_width_;

  bool reversed_border_;
  bool normal_border_;
  size_t min_cluster_pixels_;
  size_t max_cluster_pixels_;
};

// Class to zero out the count (and clear the starting offset) for each extents
// which is going to be filtered out.  Used in conjunction with SumPoints to
// update zero sized blobs.
struct TransformZeroFilteredBlobSizes {
 public:
  TransformZeroFilteredBlobSizes(size_t tag_width, bool reversed_border,
                                 bool normal_border, size_t min_cluster_pixels,
                                 size_t max_cluster_pixels)
      : select_blobs_(nullptr, tag_width, reversed_border, normal_border,
                      min_cluster_pixels, max_cluster_pixels) {}

  __host__ __device__ __forceinline__ cub::KeyValuePair<long, MinMaxExtents>
  operator()(cub::KeyValuePair<long, MinMaxExtents> pt) const {
    pt.value.count *= select_blobs_(pt.value);
    pt.value.starting_offset = 0;
    return pt;
  }

 private:
  SelectBlobs select_blobs_;
};

// Class to implement a custom Scan operator which passes through the previous
// min/max/etc, but re-sums count into starting_offset.  This lets us collapse
// out regions which don't pass the minimum filters.
struct SumPoints {
  __host__ __device__ __forceinline__ cub::KeyValuePair<long, MinMaxExtents>
  operator()(const cub::KeyValuePair<long, MinMaxExtents> &a,
             const cub::KeyValuePair<long, MinMaxExtents> &b) const {
    cub::KeyValuePair<long, MinMaxExtents> result;
    if (a.key < b.key) {
      result.value.min_x = b.value.min_x;
      result.value.min_y = b.value.min_y;
      result.value.max_x = b.value.max_x;
      result.value.max_y = b.value.max_y;
      result.value.count = b.value.count;
      result.key = b.key;
    } else {
      result.value.min_x = a.value.min_x;
      result.value.min_y = a.value.min_y;
      result.value.max_x = a.value.max_x;
      result.value.max_y = a.value.max_y;
      result.value.count = a.value.count;
      result.key = a.key;
    }

    result.value.starting_offset = a.value.starting_offset +
                                   b.value.starting_offset + a.value.count +
                                   b.value.count - result.value.count;

    return result;
  }
};

struct TransformLineFitPoint {
  __host__ __device__ __forceinline__ LineFitPoint
  operator()(IndexPoint p) const {
    LineFitPoint result;

    // we now undo our fixed-point arithmetic.
    // adjust for pixel center bias
    constexpr int delta = 1;
    int32_t ix2 = p.x() + delta;
    int32_t iy2 = p.y() + delta;
    int32_t ix = ix2 / 2;
    int32_t iy = iy2 / 2;

    int32_t W = 1;

    if (ix > 0 && ix + 1 < decimated_width && iy > 0 &&
        iy + 1 < decimated_height) {
      int32_t grad_x = decimated_image_device_[iy * decimated_width + ix + 1] -
                       decimated_image_device_[iy * decimated_width + ix - 1];

      int32_t grad_y =
          decimated_image_device_[(iy + 1) * decimated_width + ix] -
          decimated_image_device_[(iy - 1) * decimated_width + ix];

      // XXX Tunable. How to shape the gradient magnitude?
      W = hypotf(grad_x, grad_y) + 1;
    }

    result.Mx = W * ix2;
    result.My = W * iy2;
    result.Mxx = W * ix2 * ix2;
    result.Mxy = W * ix2 * iy2;
    result.Myy = W * iy2 * iy2;
    result.W = W;
    result.blob_index = p.blob_index();
    return result;
  }
  TransformLineFitPoint(const uint8_t *decimated_image_device, int decimated_width_param,
                        int decimated_height_param)
      : decimated_image_device_(decimated_image_device),
        decimated_width(decimated_width_param),
        decimated_height(decimated_height_param) {}
  const uint8_t *decimated_image_device_;
  int decimated_width;
  int decimated_height;
};

struct SumLineFitPoints {
  __host__ __device__ __forceinline__ LineFitPoint
  operator()(const LineFitPoint &a, const LineFitPoint &b) const {
    LineFitPoint result;
    result.Mx = a.Mx + b.Mx;
    result.My = a.My + b.My;
    result.Mxx = a.Mxx + b.Mxx;
    result.Mxy = a.Mxy + b.Mxy;
    result.Myy = a.Myy + b.Myy;
    result.W = a.W + b.W;
    result.blob_index = a.blob_index;
    return result;
  }
};

struct ValidPeaks {
  __host__ __device__ __forceinline__ bool operator()(const Peak &a) const {
    return a.blob_index != Peak::kNoPeak();
  }
};

struct TransformToPeakExtents {
  __host__ __device__ __forceinline__ PeakExtents
  operator()(const cub::KeyValuePair<long, Peak> &a) const {
    PeakExtents result;
    result.blob_index = a.value.blob_index;
    result.starting_offset = a.key;
    result.count = 1;
    return result;
  }
};

struct MaskPeakExtentsByBlobId {
  __host__ __device__ __forceinline__ uint32_t operator()(const Peak &a) const {
    return a.blob_index;
  }
};

struct MergePeakExtents {
  __host__ __device__ __forceinline__ PeakExtents
  operator()(const PeakExtents &a, const PeakExtents &b) const {
    PeakExtents result;
    result.blob_index = a.blob_index;
    result.starting_offset = std::min(a.starting_offset, b.starting_offset);
    result.count = a.count + b.count;
    return result;
  }
};

}  // namespace

template <InputFormat INPUT_FORMAT>
void GpuDetector<INPUT_FORMAT>::Detect(const uint8_t *image) {
  const aos::monotonic_clock::time_point start_time =
      aos::monotonic_clock::now();
  ScopedEventTiming e2e_timing(event_timings_, "e2e", stream_.get());
  start_.Record(&stream_);
  event_timings_.start("image_memcpy_to_device", stream_.get());
  // Note - since image isn't declared using host memory this
  // memcpy will not truly be async.  Experiement with copying
  // from the input image to a host memory buffer and then
  // copying from the host memory buffer to the device memory
  // buffer to see if there's a speedup.
  color_image_device_.MemcpyAsyncFrom(image, input_size_, &stream_);
  event_timings_.end("image_memcpy_to_device");
  after_image_memcpy_to_device_.Record(&stream_);

  // Threshold the image.
  event_timings_.start("CudaToGreyscaleAndDecimateHalide", stream_.get());
  CudaToGreyscaleAndDecimateHalide<INPUT_FORMAT>(
      color_image_device_.get(), gray_image_device_.get(),
      decimated_image_device_.get(), unfiltered_minmax_image_device_.get(),
      minmax_image_device_.get(), thresholded_image_device_.get(), width_,
      height_, tag_detector_->qtp.min_white_black_diff, &stream_);
  event_timings_.end("CudaToGreyscaleAndDecimateHalide");
  after_threshold_.Record(&stream_);
  event_timings_.start("gray_image_memcpy_to_host", stream_.get());
  // TODO : maybe run me on separate stream
  gray_image_device_.MemcpyAsyncTo(&gray_image_host_, &stream_);
  event_timings_.end("gray_image_memcpy_to_host");

  after_memcpy_gray_.Record(&stream_);

  event_timings_.start("union_markers_size_devize_memset", stream_.get());
  union_markers_size_device_.MemsetAsync(0u, &stream_);
  event_timings_.end("union_markers_size_devize_memset");
  after_memset_.Record(&stream_);

  // Unionfind the image.
  event_timings_.start("LabelImage", stream_.get());
  LabelImage(ToGpuImage(thresholded_image_device_),
             ToGpuImage(union_markers_device_),
             ToGpuImage(union_markers_size_device_), stream_.get());
  event_timings_.end("LabelImage");

  after_unionfinding_.Record(&stream_);

  CHECK((width_ % 8) == 0);
  CHECK((height_ % 8) == 0);

  size_t decimated_width = width_ / 2;
  size_t decimated_height = height_ / 2;

  // TODO(austin): Tune for the global shutter camera.
  // 1280 -> 2 * 128 * 5
  // 720 -> 2 * 8 * 5 * 9

  // Compute the unfiltered list of blob pairs and points.
  {
    ScopedEventTiming t(event_timings_, "BlobDiff", stream_.get());
    constexpr size_t kBlockWidth = 32;
    constexpr size_t kBlockHeight = 16;
    dim3 threads(kBlockWidth, kBlockHeight, 1);
    // Overlap 1 on each side in x, and 1 in y.
    dim3 blocks((decimated_width + threads.x - 3) / (threads.x - 2),
                (decimated_height + threads.y - 2) / (threads.y - 1), 1);

    //  Make sure we fit in our mask.
    CHECK_LT(width_ * height_, static_cast<size_t>(1 << 22));

    BlobDiff<kBlockWidth, kBlockHeight><<<blocks, threads, 0, stream_.get()>>>(
        thresholded_image_device_.get(), union_markers_device_.get(),
        union_markers_size_device_.get(), union_marker_pair_device_.get(),
        decimated_width, decimated_height);
    MaybeCheckAndSynchronize("BlobDiff");
  }

  // TODO(austin): Can I do the first step of the zero removal in the BlobDiff
  // kernel?

  after_diff_.Record(&stream_);

  // TODO - allocate this as HostMemory so we can do async copies of it
  //        Same for all subsquent uses of this pattern
  int num_compressed_union_marker_pair_host;
  {
    event_timings_.start("IfUnionMarkerPair", stream_.get());
    // Remove empty points which aren't to be considered before sorting to speed
    // things up.
    size_t temp_storage_bytes =
        temp_storage_compressed_union_marker_pair_device_.size();
    NonZero nz;
    CHECK_CUDA(cub::DeviceSelect::If(
        temp_storage_compressed_union_marker_pair_device_.get(),
        temp_storage_bytes, union_marker_pair_device_.get(),
        compressed_union_marker_pair_device_.get(),
        num_compressed_union_marker_pair_device_.get(),
        union_marker_pair_device_.size(), nz, stream_.get()));

    MaybeCheckAndSynchronize("cub::DeviceSelect::If");
    event_timings_.end("IfUnionMarkerPair");
    event_timings_.start("num_compressed_union_marker_pair_memcpy_d2h", stream_.get());
    // TODO - as an experiment, make this an async d2h copy. Add an event
    // just after it and a sync just before the data is used in the block
    // below. There's not a lot that will run in parallel on the CPU between
    // the record and sync, but there's occasionally speedups to be had by
    // not having an implicit sync.
    num_compressed_union_marker_pair_device_.MemcpyTo(
        &num_compressed_union_marker_pair_host);
    event_timings_.end("num_compressed_union_marker_pair_memcpy_d2h");
  }

  after_compact_.Record(&stream_);

  {
    ScopedEventTiming t(event_timings_, "SortUnionMarker", stream_.get());
    CHECK_LT(static_cast<size_t>(num_compressed_union_marker_pair_host),
             union_marker_pair_device_.size());

    // Now, sort just the keys to group like points.
    size_t temp_storage_bytes = radix_sort_tmpstorage_device_.size();
    QuadBoundaryPointDecomposer decomposer;
    CHECK_CUDA(cub::DeviceRadixSort::SortKeys(
        radix_sort_tmpstorage_device_.get(), temp_storage_bytes,
        compressed_union_marker_pair_device_.get(),
        sorted_union_marker_pair_device_.get(),
        num_compressed_union_marker_pair_host, decomposer,
        QuadBoundaryPoint::kRepEndBit, QuadBoundaryPoint::kBitsInKey,
        stream_.get()));

    MaybeCheckAndSynchronize("cub::DeviceRadixSort::SortKeys");
  }

  after_sort_.Record(&stream_);

  size_t num_quads_host = 0;
  {
    event_timings_.start("ReduceQuadBoundary", stream_.get());
    // Our next step is to compute the extents and dot product so we can filter
    // blobs.
    cub::ArgIndexInputIterator<QuadBoundaryPoint *> value_index_input_iterator(
        sorted_union_marker_pair_device_.get());
    TransformQuadBoundaryPointToMinMaxExtents min_max;
    cub::TransformInputIterator<MinMaxExtents,
                                TransformQuadBoundaryPointToMinMaxExtents,
                                cub::ArgIndexInputIterator<QuadBoundaryPoint *>>
        value_input_iterator(value_index_input_iterator, min_max);

    // Don't care about the output keys...
    cub::DiscardOutputIterator<uint64_t> key_discard_iterator;

    // Provide a mask to detect keys by rep01()
    MaskRep01 mask;
    cub::TransformInputIterator<uint64_t, MaskRep01, QuadBoundaryPoint *>
        key_input_iterator(sorted_union_marker_pair_device_.get(), mask);

    // Reduction operator.
    QuadBoundaryPointExtents reduce;

    size_t temp_storage_bytes =
        temp_storage_bounds_reduce_by_key_device_.size();
    cub::DeviceReduce::ReduceByKey(
        temp_storage_bounds_reduce_by_key_device_.get(), temp_storage_bytes,
        key_input_iterator, key_discard_iterator, value_input_iterator,
        extents_device_.get(), num_quads_device_.get(), reduce,
        num_compressed_union_marker_pair_host, stream_.get());
    after_bounds_.Record(&stream_);
    event_timings_.end("ReduceQuadBoundary");

    event_timings_.start("num_quads_memcpy_d2h", stream_.get());
    // TODO - as an experiment, make this an async d2h copy. Add an event
    // just after it and a sync just before the data is used in the block
    // below. There's not a lot that will run in parallel on the CPU between
    // the record and sync, but there's occasionally speedups to be had by
    // not having an implicit sync.
    num_quads_device_.MemcpyTo(&num_quads_host);
    event_timings_.end("num_quads_memcpy_d2h");
  }

  // Longest april tag will be the full perimeter of the image.  Each point
  // results in 2 neighbor points, 1 straight, and one at 45 degrees.  But, we
  // are in decimated space here, and width_ and height_ are in full image
  // space.  And there are 2 sides to each rectangle...
  //
  // Aprilrobotics has a *3 instead of a *2 here since they have duplicated
  // points in their list at this stage.
  const size_t max_april_tag_perimeter = 2 * (width_ + height_);

  {
    ScopedEventTiming t(event_timings_, "InclusiveScanByKeyMaskBlobSizes", stream_.get());
    // Now that we have the dot products, we need to rewrite the extents for the
    // post-thresholded world so we can find the start and end address of blobs
    // for fitting lines.
    //
    // Clear the size of non-passing extents and the starting offset of all
    // extents.
    cub::ArgIndexInputIterator<MinMaxExtents *> value_index_input_iterator(
        extents_device_.get());
    TransformZeroFilteredBlobSizes rewrite(
        min_tag_width_, reversed_border_, normal_border_,
        tag_detector_->qtp.min_cluster_pixels, max_april_tag_perimeter);
    cub::TransformInputIterator<cub::KeyValuePair<long, MinMaxExtents>,
                                TransformZeroFilteredBlobSizes,
                                cub::ArgIndexInputIterator<MinMaxExtents *>>
        input_iterator(value_index_input_iterator, rewrite);

    // Sum the counts of everything before us, and update the offset.
    SumPoints sum_points;

    // TODO(justin): Rip the key off when writing.

    // Rewrite the extents to have the starting offset and count match the
    // post-selected values.
    size_t temp_storage_bytes =
        temp_storage_selected_extents_scan_device_.size();
    CHECK_CUDA(cub::DeviceScan::InclusiveScan(
        temp_storage_selected_extents_scan_device_.get(), temp_storage_bytes,
        input_iterator, selected_extents_device_.get(), sum_points,
        num_quads_host, stream_.get()));

    MaybeCheckAndSynchronize("cub::DeviceScan::InclusiveScan");
  }

  after_transform_extents_.Record(&stream_);

  int num_selected_blobs_host;
  {
    event_timings_.start("SelectIndexPoints", stream_.get());
    // Now, copy over all points which pass our thresholds.
    cub::ArgIndexInputIterator<QuadBoundaryPoint *> value_index_input_iterator(
        sorted_union_marker_pair_device_.get());
    RewriteToIndexPoint rewrite(extents_device_.get(), num_quads_host);

    cub::TransformInputIterator<IndexPoint, RewriteToIndexPoint,
                                cub::ArgIndexInputIterator<QuadBoundaryPoint *>>
        input_iterator(value_index_input_iterator, rewrite);

    AddThetaToIndexPoint add_theta(extents_device_.get(), num_quads_host);

    TransformOutputIterator<IndexPoint, IndexPoint, AddThetaToIndexPoint>
        output_iterator(selected_blobs_device_.get(), add_theta);

    NonzeroBlobs select_blobs(selected_extents_device_.get());

    size_t temp_storage_bytes =
        temp_storage_compressed_filtered_blobs_device_.size();

    CHECK_CUDA(cub::DeviceSelect::If(
        temp_storage_compressed_filtered_blobs_device_.get(),
        temp_storage_bytes, input_iterator, output_iterator,
        num_selected_blobs_device_.get(), num_compressed_union_marker_pair_host,
        select_blobs, stream_.get()));
    event_timings_.end("SelectIndexPoints");

    MaybeCheckAndSynchronize("cub::DeviceSelect::If");

    event_timings_.start("num_selected_blobs_memcpy_d2h", stream_.get());
    num_selected_blobs_device_.MemcpyAsyncTo(&num_selected_blobs_host,
                                             &stream_);
    after_filter_.Record(&stream_);
    after_filter_.Synchronize();
    event_timings_.end("num_selected_blobs_memcpy_d2h");
  }

  {
    ScopedEventTiming t(event_timings_, "cub::DeviceRadixSort::SortKeysBlobs", stream_.get());
    // Sort based on the angle.
    size_t temp_storage_bytes = radix_sort_tmpstorage_device_.size();
    QuadIndexPointDecomposer decomposer;

    after_filter_.Wait(&stream_);
    CHECK_CUDA(cub::DeviceRadixSort::SortKeys(
        radix_sort_tmpstorage_device_.get(), temp_storage_bytes,
        selected_blobs_device_.get(), sorted_selected_blobs_device_.get(),
        num_selected_blobs_host, decomposer, IndexPoint::kRepEndBit,
        IndexPoint::kBitsInKey, stream_.get()));

    MaybeCheckAndSynchronize("cub::DeviceRadixSort::SortKeys");
  }

  after_filtered_sort_.Record(&stream_);

  {
    ScopedEventTiming t(event_timings_, "cub::InclusiveScanMaskBlob", stream_.get());
    // Now that we have the dot products, we need to rewrite the extents for the
    // post-thresholded world so we can find the start and end address of blobs
    // for fitting lines.
    //
    // Clear the size of non-passing extents and the starting offset of all
    // extents.
    TransformLineFitPoint rewrite(decimated_image_device_.get(), width_ / 2,
                                  height_ / 2);
    cub::TransformInputIterator<LineFitPoint, TransformLineFitPoint,
                                IndexPoint *>
        input_iterator(sorted_selected_blobs_device_.get(), rewrite);
    MaskBlobIndex mask;
    cub::TransformInputIterator<uint32_t, MaskBlobIndex, IndexPoint *>
        key_iterator(sorted_selected_blobs_device_.get(), mask);

    // Sum the counts of everything before us, and update the offset.
    SumLineFitPoints sum_points;

    // Rewrite the extents to have the starting offset and count match the
    // post-selected values.
    size_t temp_storage_bytes = temp_storage_line_fit_scan_device_.size();

    CHECK_CUDA(cub::DeviceScan::InclusiveScanByKey(
        temp_storage_line_fit_scan_device_.get(), temp_storage_bytes,
        key_iterator, input_iterator, line_fit_points_device_.get(), sum_points,
        num_selected_blobs_host, cub::Equality(), stream_.get()));

    MaybeCheckAndSynchronize("cub::DeviceScan::InclusiveScanByKey");
  }
  after_line_fit_.Record(&stream_);

  {
    ScopedEventTiming t(event_timings_, "FitLines", stream_.get());
    FitLines(line_fit_points_device_.get(), num_selected_blobs_host,
             selected_extents_device_.get(), num_quads_host, errs_device_.get(),
             filtered_errs_device_.get(), filtered_is_local_peak_device_.get(),
             &stream_);
  }
  after_line_filter_.Record(&stream_);

  int num_compressed_peaks_host;
  {
    event_timings_.start("cub::DeviceSelect::IfPeaks", stream_.get());
    // Remove empty points which aren't to be considered before sorting to speed
    // things up.
    size_t temp_storage_bytes =
        temp_storage_compressed_union_marker_pair_device_.size();
    ValidPeaks peak_filter;
    CHECK_CUDA(cub::DeviceSelect::If(
        temp_storage_compressed_union_marker_pair_device_.get(),
        temp_storage_bytes, filtered_is_local_peak_device_.get(),
        compressed_peaks_device_.get(), num_compressed_peaks_device_.get(),
        num_selected_blobs_host, peak_filter, stream_.get()));

    after_peak_compression_.Record(&stream_);
    MaybeCheckAndSynchronize("cub::DeviceSelect::If");
    event_timings_.end("cub::DeviceSelect::IfPeaks");
    event_timings_.start("num_compressed_peaks_memcpy_d2h", stream_.get());
    num_compressed_peaks_device_.MemcpyAsyncTo(&num_compressed_peaks_host,
                                               &stream_);
    after_peak_count_memcpy_.Record(&stream_);
    after_peak_count_memcpy_.Synchronize();
    event_timings_.end("num_compressed_peaks_memcpy_d2h");
  }

  {
    ScopedEventTiming t(event_timings_, "SortKeysPeaks", stream_.get());
    // Sort based on the angle.
    size_t temp_storage_bytes = radix_sort_tmpstorage_device_.size();
    PeakDecomposer decomposer;

    CHECK_CUDA(cub::DeviceRadixSort::SortKeys(
        radix_sort_tmpstorage_device_.get(), temp_storage_bytes,
        compressed_peaks_device_.get(), sorted_compressed_peaks_device_.get(),
        num_compressed_peaks_host, decomposer, 0, PeakDecomposer::kBitsInKey,
        stream_.get()));

    MaybeCheckAndSynchronize("cub::DeviceRadixSort::SortKeys");
  }

  after_peak_sort_.Record(&stream_);

  int num_quad_peaked_quads_host;
  // Now that we have the peaks sorted, recompute the extents so we can easily
  // pick out the number and top 10 peaks for line fitting.
  {
    event_timings_.start("ReduceByKeyPeakExtents", stream_.get());
    // Our next step is to compute the extents of each blob so we can filter
    // blobs.
    cub::ArgIndexInputIterator<Peak *> value_index_input_iterator(
        sorted_compressed_peaks_device_.get());
    TransformToPeakExtents transform_extents;
    cub::TransformInputIterator<PeakExtents, TransformToPeakExtents,
                                cub::ArgIndexInputIterator<Peak *>>
        value_input_iterator(value_index_input_iterator, transform_extents);

    // Don't care about the output keys...
    cub::DiscardOutputIterator<uint32_t> key_discard_iterator;

    // Provide a mask to detect keys by rep01()
    MaskPeakExtentsByBlobId mask;
    cub::TransformInputIterator<uint32_t, MaskPeakExtentsByBlobId, Peak *>
        key_input_iterator(sorted_compressed_peaks_device_.get(), mask);

    // Reduction operator.
    MergePeakExtents reduce;

    size_t temp_storage_bytes =
        temp_storage_bounds_reduce_by_key_device_.size();
    cub::DeviceReduce::ReduceByKey(
        temp_storage_bounds_reduce_by_key_device_.get(), temp_storage_bytes,
        key_input_iterator, key_discard_iterator, value_input_iterator,
        peak_extents_device_.get(), num_quad_peaked_quads_device_.get(), reduce,
        num_compressed_peaks_host, stream_.get());
    MaybeCheckAndSynchronize("cub::DeviceReduce::ReduceByKey");

    after_filtered_peak_reduce_.Record(&stream_);
    event_timings_.end("ReduceByKeyPeakExtents");

    event_timings_.start("num_quad_peaked_quads_memcpy_d2h", stream_.get());
    num_quad_peaked_quads_device_.MemcpyAsyncTo(&num_quad_peaked_quads_host,
                                                &stream_);
    MaybeCheckAndSynchronize("num_quad_peaked_quads_device_.MemcpyTo");
    after_filtered_peak_host_memcpy_.Record(&stream_);
    after_filtered_peak_host_memcpy_.Synchronize();
    event_timings_.end("num_quad_peaked_quads_memcpy_d2h");
  }

  {
    ScopedEventTiming t(event_timings_, "FitQuads", stream_.get());
    apriltag::FitQuads(
        sorted_compressed_peaks_device_.get(), num_compressed_peaks_host,
        peak_extents_device_.get(), num_quad_peaked_quads_host,
        line_fit_points_device_.get(), tag_detector_->qtp.max_nmaxima,
        selected_extents_device_.get(), tag_detector_->qtp.max_line_fit_mse,
        tag_detector_->qtp.cos_critical_rad, fit_quads_device_.get(), &stream_);
    MaybeCheckAndSynchronize("FitQuads");
  }
  after_quad_fit_.Record(&stream_);

  {
    ScopedEventTiming t(event_timings_, "MemcpyFitQuads", stream_.get());
    fit_quads_host_.resize(num_quad_peaked_quads_host);
    fit_quads_device_.MemcpyAsyncTo(fit_quads_host_.data(),
                                    num_quad_peaked_quads_host, &stream_);
    after_quad_fit_memcpy_.Record(&stream_);
    after_quad_fit_memcpy_.Synchronize();
  }
  const aos::monotonic_clock::time_point before_fit_quads =
      aos::monotonic_clock::now();
  event_timings_.start("UpdateFitQuads", stream_.get());
  UpdateFitQuads();
  event_timings_.end("UpdateFitQuads");
  event_timings_.start("AdjustPixelCenters", stream_.get());
  AdjustPixelCenters();
  event_timings_.end("AdjustPixelCenters");

  event_timings_.start("DecodeTags", stream_.get());
  DecodeTags();
  event_timings_.end("DecodeTags");

  const aos::monotonic_clock::time_point end_time = aos::monotonic_clock::now();

  // TODO(austin): Bring it back to the CPU and see how good we did.

#if 0
  // Report out how long things took.

  VLOG(1) << "Found " << num_compressed_union_marker_pair_host << " items";
  VLOG(1) << "Selected " << num_selected_blobs_host << " right side out points";
  VLOG(1) << "Found compressed runs: " << num_quads_host;
  VLOG(1) << "Peaks " << num_compressed_peaks_host << " peaks";
  VLOG(1) << "Peak Selected blobs " << num_quad_peaked_quads_host << " quads";
  CudaEvent *previous_event = &start_;
  for (auto name_event : std::vector<std::tuple<std::string_view, CudaEvent &>>{
           {"Memcpy", after_image_memcpy_to_device_},
           {"Threshold", after_threshold_},
           {"Memcpy Gray", after_memcpy_gray_},
           {"Memset", after_memset_},
           {"Unionfinding", after_unionfinding_},
           {"Diff", after_diff_},
           {"Compact", after_compact_},
           {"Sort", after_sort_},
           {"Bounds", after_bounds_},
           {"Transform Extents", after_transform_extents_},
           {"Filter by dot product", after_filter_},
           {"Filtered sort", after_filtered_sort_},
           {"Line Fit", after_line_fit_},
           {"Error Filter", after_line_filter_},
           {"Compress Peaks", after_peak_compression_},
           {"Memcpy Peaks", after_peak_count_memcpy_},
           {"Sort Peaks", after_peak_sort_},
           {"Peak Extents", after_filtered_peak_reduce_},
           {"Memcpy num Extents", after_filtered_peak_host_memcpy_},
           {"FitQuads", after_quad_fit_},
       }) {
    std::get<1>(name_event).Synchronize();
    VLOG(1) << "    " << std::get<0>(name_event) << " "
            << float_milli(std::get<1>(name_event).ElapsedTime(*previous_event))
                   .count()
            << "ms";
    previous_event = &std::get<1>(name_event);
  }
  VLOG(1) << "  FitQuads " << float_milli(end_time - before_fit_quads).count()
          << "ms on host";

  VLOG(1) << "Overall "
          << float_milli(previous_event->ElapsedTime(start_)).count() << "ms, "
          << float_milli(end_time - start_time).count() << "ms on host";
  // Average.  Skip the first one as the kernel is warming up and is slower.
  if (!first_) {
    ++execution_count_;
    execution_duration_ += previous_event->ElapsedTime(start_);
    VLOG(1) << "Average overall "
            << float_milli(execution_duration_ / execution_count_).count()
            << "ms";
  }
#endif

  first_ = false;
}

template class GpuDetector<InputFormat::Mono8>;
template class GpuDetector<InputFormat::YCbCr422>;
template class GpuDetector<InputFormat::BGR8>;

}  // namespace frc971::apriltag
