#ifndef FRC971_ORIN_APRILTAG_H_
#define FRC971_ORIN_APRILTAG_H_

#include <cub/iterator/transform_input_iterator.cuh>

#include "third_party/apriltag/apriltag.h"

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "frc971/orin/apriltag_input_format.h"
#include "frc971/orin/cuda.h"
#include "frc971/orin/cuda_event_timing.h"
#include "frc971/orin/gpu_image.h"
#include "frc971/orin/line_fit_filter.h"
#include "frc971/orin/points.h"
#include "frc971/orin/threshold.h"

namespace frc971::apriltag {

// Class to find the blob index of a point in a point vector.
class BlobExtentsIndexFinder {
 public:
  BlobExtentsIndexFinder(const MinMaxExtents *extents_device,
                         uint32_t num_extents)
      : extents_device_(extents_device), num_extents_(num_extents) {}

  __host__ __device__ uint32_t FindBlobIndex(uint32_t point_index) const {
    // Do a binary search for the blob which has the point in it's
    // starting_offset range.
    uint32_t min = 0;
    uint32_t max = num_extents_;
    while (true) {
      if (min + 1 == max) {
        return min;
      }

      uint32_t average = min + (max - min) / 2;
      if (average < num_extents_ && extents_device_[average].starting_offset <=
                                        point_index) {
        min = average;
      } else {
        max = average;
      }
    }
  }

  // Returns the extents for a blob index.
  __host__ __device__ MinMaxExtents Get(uint32_t index) const {
    return extents_device_[index];
  }

 private:
  const MinMaxExtents *extents_device_;
  uint32_t num_extents_;

  // TODO(austin): Cache the last one?
};

struct QuadCorners {
  float corners[4][2];
  bool reversed_border;
  uint32_t blob_index;
};

struct CameraMatrix {
  double fx;
  double cx;
  double fy;
  double cy;
};

struct DistCoeffs {
  double k1;
  double k2;
  double p1;
  double p2;
  double k3;
};

// GPU based april tag detector.
class GpuDetector {
 public:
  // The number of blobs we will consider when counting april tags.
  static constexpr size_t kMaxBlobs = IndexPoint::kMaxBlobs;

  // Constructs a detector, reserving space for detecting tags of the provided
  // with and height, using the provided detector options.
  GpuDetector(size_t width, size_t height, apriltag_detector_t *tag_detector,
              CameraMatrix camera_matrix, DistCoeffs distortion_coefficients,
              InputFormat input_format);
  virtual ~GpuDetector();

  // Detects april tags in the provided image.
  void Detect(const uint8_t *image);

  const std::vector<QuadCorners> &FitQuads() const;

  const zarray_t *Detections() const { return detections_; }

  // Debug methods to expose internal state for testing.
  void CopyGrayTo(uint8_t *output) const {
    gray_image_device_.MemcpyTo(output);
  }
  void CopyDecimatedTo(uint8_t *output) const {
    decimated_image_device_.MemcpyTo(output);
  }
  void CopyThresholdedTo(uint8_t *output) const {
    thresholded_image_device_.MemcpyTo(output);
  }
  void CopyUnionMarkersTo(uint32_t *output) const {
    union_markers_device_.MemcpyTo(output);
  }

  void CopyUnionMarkerPairTo(QuadBoundaryPoint *output) const {
    union_marker_pair_device_.MemcpyTo(output);
  }

  void CopyCompressedUnionMarkerPairTo(QuadBoundaryPoint *output) const {
    compressed_union_marker_pair_device_.MemcpyTo(output);
  }

  std::vector<QuadBoundaryPoint> CopySortedUnionMarkerPair() const {
    std::vector<QuadBoundaryPoint> result;
    int size = NumCompressedUnionMarkerPairs();
    result.resize(size);
    sorted_union_marker_pair_device_.MemcpyTo(result.data(), size);
    return result;
  }

  int NumCompressedUnionMarkerPairs() const {
    return num_compressed_union_marker_pair_device_.Copy()[0];
  }

  void CopyUnionMarkersSizeTo(uint32_t *output) const {
    union_markers_size_device_.MemcpyTo(output);
  }

  int NumQuads() const { return num_quads_device_.Copy()[0]; }

  std::vector<MinMaxExtents> CopyExtents() const {
    return extents_device_.Copy(NumQuads());
  }

  std::vector<cub::KeyValuePair<long, MinMaxExtents>> CopySelectedExtents()
      const {
    return selected_extents_device_.Copy(NumQuads());
  }

  int NumSelectedPairs() const { return num_selected_blobs_device_.Copy()[0]; }

  std::vector<IndexPoint> CopySelectedBlobs() const {
    return selected_blobs_device_.Copy(NumSelectedPairs());
  }

  std::vector<IndexPoint> CopySortedSelectedBlobs() const {
    return sorted_selected_blobs_device_.Copy(NumSelectedPairs());
  }

  std::vector<LineFitPoint> CopyLineFitPoints() const {
    return line_fit_points_device_.Copy(NumSelectedPairs());
  }

  std::vector<double> CopyErrors() const {
    return errs_device_.Copy(NumSelectedPairs());
  }

  std::vector<double> CopyFilteredErrors() const {
    return filtered_errs_device_.Copy(NumSelectedPairs());
  }
  std::vector<Peak> CopyPeaks() const {
    return filtered_is_local_peak_device_.Copy(NumSelectedPairs());
  }

  int NumCompressedPeaks() const {
    return num_compressed_peaks_device_.Copy()[0];
  }

  std::vector<Peak> CopyCompressedPeaks() const {
    return compressed_peaks_device_.Copy(NumCompressedPeaks());
  }

  int NumFitQuads() const { return num_quad_peaked_quads_device_.Copy()[0]; }

  std::vector<FitQuad> CopyFitQuads() const {
    return fit_quads_device_.Copy(NumFitQuads());
  }

  void AdjustCenter(float corners[4][2]) const;

  // TODO(max): We probably don't want to use these after our test images are
  // just orin images
  void SetCameraMatrix(CameraMatrix camera_matrix) {
    camera_matrix_ = camera_matrix;
  }

  void SetDistortionCoefficients(DistCoeffs distortion_coefficients) {
    distortion_coefficients_ = distortion_coefficients;
  }

  // Undistort pixels based on our camera model, using iterative algorithm
  // Returns false if we fail to converge
  // Make this a free function rather than a static member function to make
  // remove the need for callers to know the template arg from GpuDetector
  static bool UnDistort(double *u, double *v, const CameraMatrix *camera_matrix,
                        const DistCoeffs *distortion_coefficients);

 private:
  void UpdateFitQuads();

  void AdjustPixelCenters();

  void DecodeTags();

  static void QuadDecodeTask(void *_u);

  // Creates a GPU image wrapped around the provided memory.
  template <typename T>
  GpuImage<T> ToGpuImage(GpuMemory<T> &memory) {
    if (memory.size() == width_ * height_) {
      return GpuImage<T>{
          .data = memory.get(),
          .rows = height_,
          .cols = width_,
          .step = width_,
      };
    } else if (memory.size() == width_ * height_ / 4) {
      return GpuImage<T>{
          .data = memory.get(),
          .rows = height_ / 2,
          .cols = width_ / 2,
          .step = width_ / 2,
      };
    } else {
      LOG(FATAL) << "Unknown image shape";
    }
  }

  // Size of the image.
  const size_t width_;
  const size_t height_;
  const size_t input_size_;

  // Detector parameters.
  apriltag_detector_t *tag_detector_;

  // Stream to operate on.
  CudaStream stream_;

  // Separate stream for the d2h copy of grayscale output
  // This way it can run in parallel with GPU compute
  CudaStream greyscale_stream_;
  CudaStream memset_stream_;

  // Events for each of the steps for timing.
  CudaEvent start_;
  CudaEvent after_image_memcpy_to_device_;
  CudaEvent after_threshold_;
  CudaEvent after_memcpy_gray_;
  CudaEvent after_memset_;
  CudaEvent after_unionfinding_;
  CudaEvent after_diff_;
  CudaEvent after_compact_;
  CudaEvent after_sort_;
  CudaEvent after_bounds_;
  CudaEvent after_num_quads_memcpy_;
  CudaEvent after_transform_extents_;
  CudaEvent after_filter_;
  CudaEvent after_filtered_sort_;
  CudaEvent after_line_fit_;
  CudaEvent after_line_filter_;
  CudaEvent after_peak_compression_;
  CudaEvent after_peak_count_memcpy_;
  CudaEvent after_peak_sort_;
  CudaEvent after_filtered_peak_reduce_;
  CudaEvent after_filtered_peak_host_memcpy_;
  CudaEvent after_quad_fit_;
  CudaEvent after_quad_fit_memcpy_;

  HostMemory<uint8_t> gray_image_host_;
  const uint8_t      *gray_image_host_ptr_;
  HostMemory<int>     num_compressed_union_marker_pair_host_;
  HostMemory<size_t>  num_quads_host_;
  HostMemory<int>     num_selected_blobs_host_;
  HostMemory<int>     num_compressed_peaks_host_;
  HostMemory<int>     num_quad_peaked_quads_host_;


  // Starting color image.
  GpuMemory<uint8_t> color_image_device_;
  // Full size gray scale image.
  GpuMemory<uint8_t> gray_image_device_;
  // Half resolution, gray, decimated image.
  GpuMemory<uint8_t> decimated_image_device_;
  // Intermediates for thresholding.
  GpuMemory<uint8_t> unfiltered_minmax_image_device_;
  GpuMemory<uint8_t> minmax_image_device_;
  GpuMemory<uint8_t> thresholded_image_device_;

  // The union markers for each pixel.
  GpuMemory<uint32_t> union_markers_device_;
  // The size of each blob.  The blob size is stored at the index of the stored
  // union marker id in union_markers_device_ aboe.
  GpuMemory<uint32_t> union_markers_size_device_;

  // Full list of boundary points, densly stored but mostly zero.
  GpuMemory<QuadBoundaryPoint> union_marker_pair_device_;
  // Unsorted list of points with 0's removed.
  GpuMemory<QuadBoundaryPoint> compressed_union_marker_pair_device_;
  // Blob representation sorted list of points.
  GpuMemory<QuadBoundaryPoint> sorted_union_marker_pair_device_;
  // Number of compressed points.
  GpuMemory<int> num_compressed_union_marker_pair_device_{
      /* allocate 1 integer...*/ 1};

  // Number of unique blob IDs.
  GpuMemory<size_t> num_quads_device_{/* allocate 1 integer...*/ 1};
  // Bounds per blob, one blob per ID.
  GpuMemory<MinMaxExtents> extents_device_;
  // Extents of all the blobs under consideration.
  GpuMemory<cub::KeyValuePair<long, MinMaxExtents>> selected_extents_device_;

  // Number of keys in selected_blobs_device_.
  GpuMemory<int> num_selected_blobs_device_{/* allocate 1 integer...*/ 1};

  // Compacted blobs which pass our threshold.
  GpuMemory<IndexPoint> selected_blobs_device_;
  // Sorted list of those points.
  GpuMemory<IndexPoint> sorted_selected_blobs_device_;

  // TODO(austin): Can we bound this better?  This is a lot of memory.
  GpuMemory<LineFitPoint> line_fit_points_device_;

  GpuMemory<double> errs_device_;
  GpuMemory<double> filtered_errs_device_;
  GpuMemory<Peak> filtered_is_local_peak_device_;
  GpuMemory<int> num_compressed_peaks_device_{/* allocate 1 integer...*/ 1};
  GpuMemory<Peak> compressed_peaks_device_;
  GpuMemory<Peak> sorted_compressed_peaks_device_;

  GpuMemory<int> num_quad_peaked_quads_device_{/* allocate 1 integer...*/ 1};
  GpuMemory<PeakExtents> peak_extents_device_;

  CameraMatrix camera_matrix_;
  DistCoeffs distortion_coefficients_;

  GpuMemory<FitQuad> fit_quads_device_;

  std::vector<FitQuad> fit_quads_host_;
  std::vector<QuadCorners> quad_corners_host_;

  // Temporary storage for each of the steps.
  // TODO(austin): Can we combine these and just use the max?
  GpuMemory<uint32_t> radix_sort_tmpstorage_device_;
  GpuMemory<uint8_t> temp_storage_compressed_union_marker_pair_device_;
  GpuMemory<uint8_t> temp_storage_bounds_reduce_by_key_device_;
  GpuMemory<uint8_t> temp_storage_dot_product_device_;
  GpuMemory<uint8_t> temp_storage_compressed_filtered_blobs_device_;
  GpuMemory<uint8_t> temp_storage_selected_extents_scan_device_;
  GpuMemory<uint8_t> temp_storage_line_fit_scan_device_;

  Timings event_timings_;

  InputFormat input_format_;
  std::unique_ptr<BaseThreshold> threshold_;

  // Cumulative duration of april tag detection.
  std::chrono::nanoseconds execution_duration_{0};
  // Number of detections.
  size_t execution_count_ = 0;
  // True if this is the first detection.
  bool first_ = true;

  // Cached quantities used for tag filtering.
  bool normal_border_ = false;
  bool reversed_border_ = false;
  int min_tag_width_ = 1000000;

  zarray_t *poly0_;
  zarray_t *poly1_;

  zarray_t *detections_ = nullptr;
};

}  // namespace frc971::apriltag

#endif  // FRC971_ORIN_APRILTAG_H_
