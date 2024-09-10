#include <numeric>
#include <random>
#include <string>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"
#include "opencv2/imgproc.hpp"
#include "third_party/apriltag/apriltag.h"
#include "third_party/apriltag/common/unionfind.h"
#include "third_party/apriltag/tag16h5.h"
#include "third_party/apriltag/tag36h11.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "aos/testing/random_seed.h"
#include "aos/time/time.h"
#include "frc971/orin/apriltag.h"
#include "frc971/vision/vision_generated.h"

ABSL_FLAG(int32_t, pixel_border, 10,
          "Size of image border within which to reject detected corners");
ABSL_FLAG(double, min_decision_margin, 50.0,
          "Minimum decision margin (confidence) for an apriltag detection");

ABSL_FLAG(bool, debug, false, "If true, write debug images.");

ABSL_DECLARE_FLAG(int32_t, debug_blob_index);

// Get access to the intermediates of aprilrobotics.
extern "C" {

image_u8_t *threshold(apriltag_detector_t *td, image_u8_t *im);
unionfind_t *connected_components(apriltag_detector_t *td, image_u8_t *threshim,
                                  int w, int h, int ts);

zarray_t *gradient_clusters(apriltag_detector_t *td, image_u8_t *threshim,
                            int w, int h, int ts, unionfind_t *uf);

struct pt {
  // Note: these represent 2*actual value.
  uint16_t x, y;
  int16_t gx, gy;

  float slope;
};

struct line_fit_pt {
  double Mx, My;
  double Mxx, Myy, Mxy;
  double W;  // total weight
};

struct line_fit_pt *compute_lfps(int sz, zarray_t *cluster, image_u8_t *im);

void fit_line(struct line_fit_pt *lfps, int sz, int i0, int i1,
              double *lineparm, double *err, double *mse);

int fit_quad(apriltag_detector_t *td, image_u8_t *im, zarray_t *cluster,
             struct quad *quad, int tag_width, bool normal_border,
             bool reversed_border);

}  // extern C

std::ostream &operator<<(std::ostream &os, const line_fit_pt &point) {
  os << "{Mx:" << std::setprecision(20) << point.Mx
     << ", My:" << std::setprecision(20) << point.My
     << ", Mxx:" << std::setprecision(20) << point.Mxx
     << ", Myy:" << std::setprecision(20) << point.Myy
     << ", Mxy:" << std::setprecision(20) << point.Mxy << ", W:" << point.W
     << "}";
  return os;
}

namespace frc971::apriltag {
std::ostream &operator<<(std::ostream &os,
                         const frc971::apriltag::LineFitPoint &point) {
  os << "{Mx:" << point.Mx / 2.0 << ", My:" << point.My / 2.0
     << ", Mxx:" << point.Mxx / 4.0 << ", Mxy:" << point.Mxy / 4.0
     << ", Myy:" << point.Myy / 4.0 << ", W:" << point.W << "}";
  return os;
}

}  // namespace frc971::apriltag

// Converts a cv::Mat to an aprilrobotics image.
image_u8_t ToImageu8t(const cv::Mat &img) {
  return image_u8_t{
      .width = img.cols,
      .height = img.rows,
      .stride = img.cols,
      .buf = img.data,
  };
}

namespace frc971::apriltag::testing {

// Checks that 2 images match.
void CheckImage(image_u8_t compare_im_one, image_u8_t compare_im_two,
                std::string_view label) {
  CHECK_EQ(compare_im_one.width, compare_im_two.width);
  CHECK_EQ(compare_im_one.height, compare_im_two.height);
  ssize_t p = 0;
  for (int j = 0; j < compare_im_one.height; ++j) {
    for (int i = 0; i < compare_im_one.width; ++i) {
      if (p < 0) {
        LOG(INFO) << i << " " << j << ": "
                  << static_cast<int>(
                         compare_im_one.buf[j * compare_im_one.stride + i])
                  << " (address + " << j * compare_im_one.stride + i << ") vs "
                  << static_cast<int>(
                         compare_im_two.buf[j * compare_im_two.stride + i])
                  << " (address + " << j * compare_im_two.stride + i << ") for "
                  << label;

        ++p;
      }

      CHECK_EQ(compare_im_one.buf[j * compare_im_one.stride + i],
               compare_im_two.buf[j * compare_im_two.stride + i])
          << "First Image Value "
          << (int)compare_im_one.buf[j * compare_im_one.stride + i] << " "
          << "Second Image Value "
          << (int)compare_im_two.buf[j * compare_im_two.stride + i] << " "
          << "At " << i << ", " << j << " for " << label;
    }
  }
}

// Stores info about a blob.
struct BlobInfo {
  size_t size;
  size_t min_x;
  size_t min_y;
  size_t max_x;
  size_t max_y;
  size_t april_robotics_id;
  size_t bounding_area() const { return (max_x - min_x) * (max_y - min_y); }
};

// Checks that the aprilrobotics and GPU code agree on the results of
// unionfinding.
std::map<uint32_t, BlobInfo> CheckUnionfind(unionfind_t *uf,
                                            cv::Mat union_markers,
                                            const uint32_t *union_markers_size,
                                            cv::Mat threshold) {
  const size_t width = union_markers.cols;
  const size_t height = union_markers.rows;
  std::map<uint32_t, uint32_t> id_remap;
  std::map<uint32_t, size_t> cuda_id_count;

  for (size_t y = 0; y < height; ++y) {
    for (size_t x = 0; x < width; ++x) {
      uint32_t v = unionfind_get_representative(uf, y * width + x);
      uint32_t v_remapped;
      {
        auto it = cuda_id_count.emplace(union_markers.at<uint32_t>(y, x), 1);
        if (!it.second) {
          ++it.first->second;
        }
      }
      auto it = id_remap.find(v);
      if (it == id_remap.end()) {
        v_remapped = union_markers.at<uint32_t>(y, x);
        id_remap.insert(std::make_pair(v, v_remapped));
      } else {
        v_remapped = it->second;
      }

      CHECK_EQ(v_remapped, union_markers.at<uint32_t>(y, x))
          << "At " << x << ", " << y;
    }
  }
  for (auto [key, value] : cuda_id_count) {
    VLOG(2) << "Found " << key << " num times " << value;
  }
  LOG(INFO) << "Found " << id_remap.size() << " blob ids in aprilrobotics.";

  std::map<uint32_t, BlobInfo> blob_sizes;

  for (size_t y = 0; y < height; ++y) {
    for (size_t x = 0; x < width; ++x) {
      auto it = blob_sizes.emplace(
          union_markers.at<uint32_t>(y, x),
          BlobInfo{
              .size = 1,
              .min_x = x,
              .min_y = y,
              .max_x = x,
              .max_y = y,
              .april_robotics_id =
                  unionfind_get_representative(uf, y * width + x),
          });
      if (!it.second) {
        BlobInfo *info = &(it.first->second);
        ++(info->size);
        CHECK_EQ(info->april_robotics_id,
                 unionfind_get_representative(uf, y * width + x));
        info->min_x = std::min(info->min_x, x);
        info->max_x = std::max(info->max_x, x);
        info->min_y = std::min(info->min_y, y);
        info->max_y = std::max(info->max_y, y);
      }
    }
  }

  for (size_t y = 0; y < height; ++y) {
    for (size_t x = 0; x < width; ++x) {
      const uint32_t blob_id = union_markers.at<uint32_t>(y, x);
      const BlobInfo &blob_info = blob_sizes[blob_id];
      if (threshold.at<uint8_t>(y, x) == 127) {
        CHECK_EQ(0u, union_markers_size[blob_id])
            << " at (" << x << ", " << y << ") -> " << blob_id;
        continue;
      }

      if (blob_info.size >= 25u) {
        CHECK_LE(25u, union_markers_size[blob_id])
            << " at (" << x << ", " << y << ") -> " << blob_id;
      } else {
        CHECK_EQ(blob_info.size, union_markers_size[blob_id])
            << " at (" << x << ", " << y << ") -> " << blob_id;
      }
    }
  }

  LOG(INFO) << "Union finding + stats passed.";

  return blob_sizes;
}

// Makes a tag detector.
apriltag_detector_t *MakeTagDetector(apriltag_family_t *tag_family) {
  apriltag_detector_t *tag_detector = apriltag_detector_create();

  apriltag_detector_add_family_bits(tag_detector, tag_family, 1);

  tag_detector->nthreads = 6;
  tag_detector->wp = workerpool_create(tag_detector->nthreads);
  tag_detector->qtp.min_white_black_diff = 5;
  tag_detector->debug = absl::GetFlag(FLAGS_debug);

  return tag_detector;
}

// TODO(max): Create a function which will take in the calibration data
CameraMatrix create_camera_matrix() {
  return CameraMatrix{
      1,
      1,
      1,
      1,
  };
}

DistCoeffs create_distortion_coefficients() {
  return DistCoeffs{
      0, 0, 0, 0, 0,
  };
}

class CudaAprilTagDetector {
 public:
  CudaAprilTagDetector(size_t width, size_t height,
                       apriltag_family_t *tag_family = tag16h5_create())
      : tag_family_(tag_family),
        tag_detector_(MakeTagDetector(tag_family_)),
        gray_cuda_(cv::Size(width, height), CV_8UC1),
        decimated_cuda_(gray_cuda_.size() / 2, CV_8UC1),
        thresholded_cuda_(decimated_cuda_.size(), CV_8UC1),
        union_markers_(decimated_cuda_.size(), CV_32SC1),
        union_markers_size_(decimated_cuda_.size(), CV_32SC1),
        gpu_detector_(width, height, tag_detector_, create_camera_matrix(),
                      create_distortion_coefficients()),
        width_(width),
        height_(height) {
    // Report out info about our GPU.
    {
      cudaDeviceProp prop;
      CHECK_EQ(cudaGetDeviceProperties(&prop, 0), cudaSuccess);

      LOG(INFO) << "Device: sm_" << prop.major << prop.minor;
#define DUMP(x) LOG(INFO) << "" #x ": " << prop.x;
      DUMP(sharedMemPerBlock);
      DUMP(l2CacheSize);
      DUMP(maxThreadsPerBlock);
      DUMP(maxThreadsPerMultiProcessor);
      DUMP(memoryBusWidth);
      DUMP(memoryClockRate);
      DUMP(multiProcessorCount);
      DUMP(maxBlocksPerMultiProcessor);
      DUMP(name);
      DUMP(warpSize);

#undef DUMP
    }

    union_marker_pair_.resize((width - 2) / 2 * (height - 2) / 2 * 4,
                              QuadBoundaryPoint());
    compressed_union_marker_pair_.resize((width - 2) / 2 * (height - 2) / 2 * 4,
                                         QuadBoundaryPoint());

    // Pre-compute the border and width thresholds.
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
  }

  ~CudaAprilTagDetector() {
    if (aprilrobotics_detections_ != nullptr) {
      apriltag_detections_destroy(aprilrobotics_detections_);
    }
    apriltag_detector_destroy(tag_detector_);
    free(tag_family_);
  }

  // Detects tags on the GPU.
  void DetectGPU(cv::Mat color_image) {
    CHECK_EQ(color_image.size(), gray_cuda_.size());

    gpu_detector_.Detect(color_image.data);

    gpu_detector_.CopyGrayTo(gray_cuda_.data);
    gpu_detector_.CopyDecimatedTo(decimated_cuda_.data);
    gpu_detector_.CopyThresholdedTo(thresholded_cuda_.data);

    gpu_detector_.CopyUnionMarkersTo(
        reinterpret_cast<uint32_t *>(union_markers_.data));
    gpu_detector_.CopyUnionMarkersSizeTo(
        reinterpret_cast<uint32_t *>(union_markers_size_.data));

    gpu_detector_.CopyUnionMarkerPairTo(union_marker_pair_.data());
    gpu_detector_.CopyCompressedUnionMarkerPairTo(
        compressed_union_marker_pair_.data());
    sorted_union_marker_pair_ = gpu_detector_.CopySortedUnionMarkerPair();
    num_compressed_union_marker_pair_ =
        gpu_detector_.NumCompressedUnionMarkerPairs();
    extents_cuda_ = gpu_detector_.CopyExtents();
    selected_extents_cuda_ = gpu_detector_.CopySelectedExtents();
    selected_blobs_cuda_ = gpu_detector_.CopySelectedBlobs();
    sorted_selected_blobs_cuda_ = gpu_detector_.CopySortedSelectedBlobs();
    line_fit_points_cuda_ = gpu_detector_.CopyLineFitPoints();
    errors_device_ = gpu_detector_.CopyErrors();
    filtered_errors_device_ = gpu_detector_.CopyFilteredErrors();
    peaks_device_ = gpu_detector_.CopyPeaks();
    num_quads_ = gpu_detector_.NumQuads();

    fit_quads_ = gpu_detector_.FitQuads();

    LOG(INFO) << "num_compressed_union_marker_pair "
              << sorted_union_marker_pair_.size();
  }

  // Detects tags on the CPU.
  void DetectCPU(cv::Mat color_image) {
    cv::Mat gray_image(color_image.size(), CV_8UC1);
    cv::cvtColor(color_image, gray_image, cv::COLOR_YUV2GRAY_YUYV);
    image_u8_t im = {
        .width = gray_image.cols,
        .height = gray_image.rows,
        .stride = gray_image.cols,
        .buf = gray_image.data,
    };

    LOG(INFO) << "Starting CPU detect.";
    if (aprilrobotics_detections_ != nullptr) {
      apriltag_detections_destroy(aprilrobotics_detections_);
    }
    aprilrobotics_detections_ = apriltag_detector_detect(tag_detector_, &im);

    timeprofile_display(tag_detector_->tp);

    for (int i = 0; i < zarray_size(aprilrobotics_detections_); i++) {
      apriltag_detection_t *det;
      zarray_get(aprilrobotics_detections_, i, &det);

      if (det->decision_margin > absl::GetFlag(FLAGS_min_decision_margin)) {
        LOG(INFO) << "Found tag number " << det->id
                  << " hamming: " << det->hamming
                  << " margin: " << det->decision_margin;
      }
    }
  }

  // Checks that the union finding algorithms agree.
  std::vector<QuadBoundaryPoint> CheckUnionMarkerPairs(
      const std::map<uint32_t, BlobInfo> &blob_sizes) const {
    std::vector<QuadBoundaryPoint> expected_union_marker_pair;

    const size_t width = thresholded_cuda_.cols;
    const size_t height = thresholded_cuda_.rows;

    expected_union_marker_pair.resize((width - 2) * (height - 2) * 4,
                                      QuadBoundaryPoint());

    LOG(INFO) << "Width: " << width << ", Height " << height;

    auto ToIndex = [&](size_t x, size_t y) {
      return x - 1 + (y - 1) * (width - 2);
    };

    size_t wrong = 0;
    size_t right_nonzero = 0;

    for (size_t y = 1; y < height - 1; ++y) {
      for (size_t x = 1; x < width - 1; ++x) {
        const uint8_t v0 = thresholded_cuda_.at<uint8_t>(y, x);
        const uint64_t rep0 = union_markers_.at<uint32_t>(y, x);
        const auto blob0 = blob_sizes.find(rep0)->second;

        size_t offset = 0;
        for (auto [dx, dy] : {std::pair<ssize_t, ssize_t>{1, 0},
                              std::pair<ssize_t, ssize_t>{1, 1},
                              std::pair<ssize_t, ssize_t>{0, 1},
                              std::pair<ssize_t, ssize_t>{-1, 1}}) {
          const uint8_t v1 = thresholded_cuda_.at<uint8_t>(y + dy, x + dx);
          const uint32_t rep1 = union_markers_.at<uint32_t>(y + dy, x + dx);
          const auto blob1 = blob_sizes.find(rep1)->second;

          // OK, blob 1, 1 is special.  We only do it if blocks 2 and the blob
          // let of x won't be connected.
          //      ________
          //      | x | 0 |
          //  -------------
          //  | 3 | 2 | 1 |
          //  -------------
          //
          //  We are fine checking this all here since we verify that we agree
          //  with aprilrobotics at the end to make sure the overall logic
          //  matches.
          bool consider_blob = true;
          if (offset == 3) {
            const uint8_t v1_block_left =
                thresholded_cuda_.at<uint8_t>(y + 0, x - 1);
            const uint32_t rep1_block_left =
                union_markers_.at<uint32_t>(y + 0, x - 1);
            const uint8_t v1_block_2 =
                thresholded_cuda_.at<uint8_t>(y + 1, x + 0);
            const uint32_t rep1_block_2 =
                union_markers_.at<uint32_t>(y + 1, x + 0);
            if (v1_block_left != 127 && v1_block_2 != 127 &&
                (v1_block_left + v1_block_2) == 255 &&
                blob_sizes.find(rep1_block_left)->second.size >= 25 &&
                blob_sizes.find(rep1_block_2)->second.size >= 25 && x != 1) {
              consider_blob = false;
            }
          }

          QuadBoundaryPoint point;
          if (consider_blob && blob0.size >= 25 && blob1.size >= 25 &&
              v0 != 127) {
            if (v0 + v1 == 255) {
              if (rep0 < rep1) {
                point.set_rep1(rep1);
                point.set_rep0(rep0);
              } else {
                point.set_rep1(rep0);
                point.set_rep0(rep1);
              }
              point.set_base_xy(x, y);
              point.set_dxy(offset);
              point.set_black_to_white(v1 > v0);
            }
          }
          size_t index = ToIndex(x, y) + offset * (width - 2) * (height - 2);

          QuadBoundaryPoint actual = union_marker_pair_[index];
          if (!point.near(actual)) {
            CHECK_LT(wrong, 10u);
            LOG(WARNING) << "point == actual (" << std::hex << point << ", "
                         << actual << ") : Failed at (" << std::dec << x << ", "
                         << y << ") + (" << dx << ", " << dy
                         << "), v0: " << static_cast<int>(v0)
                         << ", v1: " << static_cast<int>(v1)
                         << ", rep0: " << std::hex << rep0 << " rep1: " << rep1
                         << " rep0 size: " << std::dec << blob0.size
                         << " rep1 size: " << std::dec << blob1.size
                         << " consider_blob " << consider_blob;
            ++wrong;
          } else if (point.nonzero()) {
            right_nonzero++;
            VLOG(2) << "point == actual (" << std::hex << point << ", "
                    << actual << ") : Success at (" << std::dec << x << ", "
                    << y << ") + (" << dx << ", " << dy
                    << "), v0: " << static_cast<int>(v0)
                    << ", v1: " << static_cast<int>(v1)
                    << ", rep0: " << std::hex << rep0 << " rep1: " << rep1;
            point = actual;
          }
          expected_union_marker_pair[index] = point;

          ++offset;
        }
      }
    }
    CHECK_EQ(wrong, 0u) << ", got " << right_nonzero << " right";

    for (size_t i = 0; i < expected_union_marker_pair.size(); ++i) {
      CHECK_EQ(expected_union_marker_pair[i], union_marker_pair_[i]);
    }

    return expected_union_marker_pair;
  }

  // Checks that the compressed marker pairs GPU algorithm agrees with a naive
  // CPU version.  Returns the expected points.
  std::vector<QuadBoundaryPoint> CheckCompressedUnionMarkerPairs(
      const std::vector<QuadBoundaryPoint> &expected_union_marker_pair,
      const std::vector<QuadBoundaryPoint> &compressed_union_marker_pair,
      const std::vector<QuadBoundaryPoint> &sorted_union_marker_pair) const {
    std::vector<QuadBoundaryPoint> expected_compressed_union_marker_pair;
    {
      size_t nonzero_count = 0;
      for (const QuadBoundaryPoint x : expected_union_marker_pair) {
        if (x.nonzero()) {
          ++nonzero_count;
        }
      }
      // Rip out all the zeros.
      expected_compressed_union_marker_pair.reserve(nonzero_count);
      for (const QuadBoundaryPoint x : expected_union_marker_pair) {
        if (x.nonzero()) {
          expected_compressed_union_marker_pair.push_back(x);
        }
      }

      CHECK_EQ(static_cast<size_t>(sorted_union_marker_pair.size()),
               expected_compressed_union_marker_pair.size());

      // And make sure they match.
      for (size_t i = 0; i < expected_compressed_union_marker_pair.size();
           ++i) {
        CHECK_EQ(expected_compressed_union_marker_pair[i],
                 compressed_union_marker_pair[i]);
      }
    }

    // Now, sort the points.
    std::sort(expected_compressed_union_marker_pair.begin(),
              expected_compressed_union_marker_pair.end(),
              [&](const QuadBoundaryPoint a, const QuadBoundaryPoint b) {
                if (a.rep01() != b.rep01()) {
                  return a.rep01() < b.rep01();
                }

                return a < b;
              });

    for (size_t i = 0; i < static_cast<size_t>(sorted_union_marker_pair.size());
         ++i) {
      CHECK_EQ(expected_compressed_union_marker_pair[i].rep01(),
               sorted_union_marker_pair[i].rep01());
    }

    return expected_compressed_union_marker_pair;
  }

  // Sorts the list of points by slope and blob ID.
  std::vector<QuadBoundaryPoint> SlopeSortPoints(
      std::vector<QuadBoundaryPoint> points) const {
    std::map<uint64_t, std::pair<float, float>> blob_centers;

    // The slope algorithm used by aprilrobotics.
    auto ComputeSlope = [&blob_centers](QuadBoundaryPoint pair) -> float {
      const int32_t x = pair.x();
      const int32_t y = pair.y();

      auto blob_center = blob_centers.find(pair.rep01());
      CHECK(blob_center != blob_centers.end());

      const float cx = blob_center->second.first;
      const float cy = blob_center->second.second;

      float quadrants[2][2] = {{-1 * (2 << 15), 0}, {2 * (2 << 15), 2 << 15}};

      float dx = x - cx;
      float dy = y - cy;

      float quadrant = quadrants[dy > 0][dx > 0];
      if (dy < 0) {
        dy = -dy;
        dx = -dx;
      }

      if (dx < 0) {
        float tmp = dx;
        dx = dy;
        dy = -tmp;
      }

      return quadrant + dy / dx;
    };

    // The theta algorithm used by cuda.
    auto ComputeTheta = [&blob_centers](QuadBoundaryPoint pair) -> long double {
      const int32_t x = pair.x();
      const int32_t y = pair.y();

      auto blob_center = blob_centers.find(pair.rep01());
      CHECK(blob_center != blob_centers.end());

      const float cx = blob_center->second.first;
      const float cy = blob_center->second.second;

      float dx = x - cx;
      float dy = y - cy;

      // make atan2 more accurate than cuda to correctly sort
      return atan2f64(dy, dx);
    };

    for (size_t i = 0; i < points.size();) {
      uint64_t first_rep01 = points[i].rep01();

      int min_x, min_y, max_x, max_y;
      min_x = max_x = points[i].x();
      min_y = max_y = points[i].y();

      size_t j = i;
      for (; j < points.size() && points[j].rep01() == first_rep01; ++j) {
        QuadBoundaryPoint pt = points[j];

        int x = pt.x();
        int y = pt.y();
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
      }

      const float cx = (max_x + min_x) * 0.5 + 0.05118;
      const float cy = (max_y + min_y) * 0.5 + -0.028581;

      blob_centers[first_rep01] = std::make_pair(cx, cy);
      i = j;
    }

    std::sort(points.begin(), points.end(),
              [&](const QuadBoundaryPoint a, const QuadBoundaryPoint b) {
                if (a.rep01() != b.rep01()) {
                  return a.rep01() < b.rep01();
                }

                const float slopea = ComputeSlope(a);
                const float slopeb = ComputeSlope(b);

                // Sigh, apparently the slope algorithm isn't great and
                // sometimes ends up with matching slopes.  In that case,
                // compute the actual angle to the X axis too and sort with it.
                //
                // Aprilrobotics ignores this case and ends up with an
                // indeterminate solution.  We don't, so we notice.
                if (slopea == slopeb) {
                  return ComputeTheta(a) < ComputeTheta(b);
                }
                return slopea < slopeb;
              });
    return points;
  }

  // Filters blobs using the the various size thresholds that aprilrobotics
  // uses.
  std::vector<std::vector<QuadBoundaryPoint>> FilterBlobs(
      std::vector<std::vector<QuadBoundaryPoint>> blobs) const {
    std::vector<std::vector<QuadBoundaryPoint>> result;
    const size_t max_april_tag_perimeter = 2 * (width_ + height_);
    LOG(ERROR) << "Max permiter test " << max_april_tag_perimeter;

    for (std::vector<QuadBoundaryPoint> &blob : blobs) {
      int min_x, min_y, max_x, max_y;
      min_x = max_x = blob[0].x();
      min_y = max_y = blob[0].y();

      for (const QuadBoundaryPoint pt : blob) {
        int x = pt.x();
        int y = pt.y();
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
      }

      float dot = 0;

      const float cx = (min_x + max_x) * 0.5 + 0.05118;
      const float cy = (min_y + max_y) * 0.5 - 0.028581;

      for (size_t j = 0; j < blob.size(); ++j) {
        dot += (static_cast<float>(blob[j].x()) - cx) * blob[j].gx() +
               (static_cast<float>(blob[j].y()) - cy) * blob[j].gy();
      }

      const bool quad_reversed_border = dot < 0;

      if (!reversed_border_ && quad_reversed_border) {
        continue;
      }
      if (!normal_border_ && !quad_reversed_border) {
        continue;
      }
      if ((max_x - min_x) * (max_y - min_y) < min_tag_width_) {
        continue;
      }
      if (blob.size() < 24) {
        continue;
      }
      if (blob.size() <
          static_cast<size_t>(tag_detector_->qtp.min_cluster_pixels)) {
        continue;
      }
      if (blob.size() > max_april_tag_perimeter) {
        continue;
      }

      result.emplace_back(std::move(blob));
    }

    return result;
  }

  // Produces sorted lists of clusters from AprilRobotics' intermediate
  // clusters.
  std::vector<std::vector<uint64_t>> AprilRoboticsPoints(
      image_u8_t *thresholded_im, unionfind_t *uf) const {
    zarray_t *clusters =
        gradient_clusters(tag_detector_, thresholded_im, thresholded_im->width,
                          thresholded_im->height, thresholded_im->stride, uf);

    std::vector<std::vector<uint64_t>> april_grouped_points;
    for (int i = 0; i < zarray_size(clusters); i++) {
      zarray_t *cluster;
      zarray_get(clusters, i, &cluster);

      uint16_t min_x = std::numeric_limits<uint16_t>::max();
      uint16_t max_x = 0;
      uint16_t min_y = std::numeric_limits<uint16_t>::max();
      uint16_t max_y = 0;
      std::vector<std::tuple<float, uint64_t>> pts;
      for (int j = 0; j < zarray_size(cluster); j++) {
        struct pt *p;
        zarray_get_volatile(cluster, j, &p);
        min_x = std::min(p->x, min_x);
        min_y = std::min(p->y, min_y);
        max_x = std::max(p->x, max_x);
        max_y = std::max(p->y, max_y);
      }

      // add some noise to (cx,cy) so that pixels get a more diverse set
      // of theta estimates. This will help us remove more points.
      // (Only helps a small amount. The actual noise values here don't
      // matter much at all, but we want them [-1, 1]. (XXX with
      // fixed-point, should range be bigger?)
      const float cx = (min_x + max_x) * 0.5 + 0.05118;
      const float cy = (min_y + max_y) * 0.5 + -0.028581;

      float quadrants[2][2] = {{-1 * (2 << 15), 0}, {2 * (2 << 15), 2 << 15}};

      for (int j = 0; j < zarray_size(cluster); j++) {
        struct pt *p;
        zarray_get_volatile(cluster, j, &p);

        float dx = p->x - cx;
        float dy = p->y - cy;

        float quadrant = quadrants[dy > 0][dx > 0];
        if (dy < 0) {
          dy = -dy;
          dx = -dx;
        }

        if (dx < 0) {
          float tmp = dx;
          dx = dy;
          dy = -tmp;
        }
        p->slope = quadrant + dy / dx;
        pts.push_back(std::make_pair(p->slope, p->x + p->y * width_));
      }

      // Slope sort doesn't always combine duplicates because of duplicate
      // slopes. Sort by point first, remove duplicates, then sort by slope.
      std::sort(
          pts.begin(), pts.end(),
          [](std::tuple<float, uint64_t> a, std::tuple<float, uint64_t> b) {
            return std::get<1>(a) < std::get<1>(b);
          });
      size_t before_size = pts.size();
      pts.erase(std::unique(pts.begin(), pts.end(),
                            [](std::tuple<float, uint64_t> a,
                               std::tuple<float, uint64_t> b) {
                              return std::get<1>(a) == std::get<1>(b);
                            }),
                pts.end());
      CHECK_EQ(pts.size(), before_size);

      std::sort(
          pts.begin(), pts.end(),
          [](std::tuple<float, uint64_t> a, std::tuple<float, uint64_t> b) {
            return std::get<0>(a) < std::get<0>(b);
          });

      VLOG(1) << "aprilrobotics points of " << pts.size() << " center (" << cx
              << ", " << cy << ")";

      std::vector<uint64_t> transformed_points;
      transformed_points.reserve(pts.size());
      for (std::tuple<float, uint64_t> pt : pts) {
        VLOG(1) << "    (" << std::get<1>(pt) % width_ << ", "
                << std::get<1>(pt) / width_ << ") slope " << std::get<0>(pt);
        transformed_points.push_back(std::get<1>(pt));
      }

      april_grouped_points.emplace_back(std::move(transformed_points));
    }

    // Now sort the groups of points into a reproducible result by sorting by
    // size and then smallest point first.
    std::sort(april_grouped_points.begin(), april_grouped_points.end(),
              [](auto &x, auto &y) {
                if (x.size() == y.size()) {
                  for (size_t j = 0; j < x.size(); ++j) {
                    if (x[j] == y[j]) {
                      continue;
                    }
                    return x[j] < y[j];
                  }
                  LOG(FATAL) << "Equal";
                }
                return x.size() < y.size();
              });

    for (int i = 0; i < zarray_size(clusters); i++) {
      zarray_t *cluster;
      zarray_get(clusters, i, &cluster);
      zarray_destroy(cluster);
    }
    zarray_destroy(clusters);

    return april_grouped_points;
  }

  // Sorts point lists by size.  These points need to have an x() and y().
  template <typename T>
  std::vector<std::vector<T>> SortPointSizes(
      std::vector<std::vector<T>> in) const {
    std::sort(in.begin(), in.end(), [this](auto &a, auto &b) {
      if (a.size() == b.size()) {
        // Use point ID as the tie breaker.
        for (size_t j = 0; j < a.size(); ++j) {
          const uint32_t point_x = a[j].x() + a[j].y() * width_;
          const uint32_t point_y = b[j].x() + b[j].y() * width_;
          if (point_x == point_y) {
            continue;
          }
          return point_x < point_y;
        }
        LOG(FATAL) << "Equal";
      }
      return a.size() < b.size();
    });

    return in;
  }

  // Sorts grouped points by size and point.
  std::vector<std::vector<QuadBoundaryPoint>> SortPoints(
      std::vector<std::vector<QuadBoundaryPoint>> in) const {
    for (std::vector<QuadBoundaryPoint> &grouped_points : in) {
      std::sort(grouped_points.begin(), grouped_points.end(),
                [this](const QuadBoundaryPoint &a, const QuadBoundaryPoint &b) {
                  return std::make_tuple(a.rep01(), a.x() + a.y() * width_) <
                         std::make_tuple(b.rep01(), b.x() + b.y() * width_);
                });
    }

    return SortPointSizes(std::move(in));
  }

  // Sorts outer points list by size and point.
  std::vector<std::vector<uint64_t>> SortAprilroboticsPointSizes(
      std::vector<std::vector<uint64_t>> april_grouped_points) const {
    std::sort(april_grouped_points.begin(), april_grouped_points.end(),
              [](auto &x, auto &y) {
                if (x.size() == y.size()) {
                  for (size_t j = 0; j < x.size(); ++j) {
                    if (x[j] == y[j]) {
                      continue;
                    }
                    return x[j] < y[j];
                  }
                  LOG(FATAL) << "Equal";
                }
                return x.size() < y.size();
              });
    return april_grouped_points;
  }

  // Sorts points and points lists for Aprilrobotics by size and point id.
  std::vector<std::vector<uint64_t>> SortAprilroboticsPoints(
      std::vector<std::vector<uint64_t>> april_grouped_points) const {
    for (std::vector<uint64_t> &points : april_grouped_points) {
      std::sort(points.begin(), points.end());
    }
    return SortAprilroboticsPointSizes(april_grouped_points);
  }

  // Prints out a list of CUDA points.
  void PrintCudaPoints(
      const std::vector<std::vector<QuadBoundaryPoint>> &cuda_grouped_points,
      const std::map<uint32_t, BlobInfo> &blob_sizes) const {
    for (const std::vector<QuadBoundaryPoint> &points : cuda_grouped_points) {
      const uint32_t rep0 = points[0].rep0();
      const uint32_t rep1 = points[0].rep1();
      VLOG(1) << "CUDA points of " << rep0 << "+" << rep1
              << " aprilrobotics blob: "
              << blob_sizes.find(rep0)->second.april_robotics_id << "+"
              << blob_sizes.find(rep1)->second.april_robotics_id << " ("
              << std::hex
              << ((static_cast<uint64_t>(std::max(
                       blob_sizes.find(rep0)->second.april_robotics_id,
                       blob_sizes.find(rep1)->second.april_robotics_id))
                   << 32) |
                  std::min(blob_sizes.find(rep0)->second.april_robotics_id,
                           blob_sizes.find(rep1)->second.april_robotics_id))
              << std::dec << ") size " << points.size();
      for (const QuadBoundaryPoint point : points) {
        VLOG(1) << "    (" << point.x() << ", " << point.y() << ")";
      }
    }

    LOG(INFO) << "Found runs overall " << cuda_grouped_points.size();
  }

  // Groups marker pairs by runs of rep01().
  std::vector<std::vector<QuadBoundaryPoint>> CudaGroupedPoints(
      const std::vector<QuadBoundaryPoint> &compressed_union_marker_pairs)
      const {
    std::vector<std::vector<QuadBoundaryPoint>> cuda_grouped_points;
    cuda_grouped_points.emplace_back();

    QuadBoundaryPoint start = compressed_union_marker_pairs[0];
    for (size_t i = 0; i < compressed_union_marker_pairs.size(); ++i) {
      QuadBoundaryPoint current = compressed_union_marker_pairs[i];
      CHECK_GE(current.rep01(), start.rep01())
          << " Failed on index " << i << " of "
          << compressed_union_marker_pairs.size();
      if ((start.rep01()) != (current.rep01())) {
        cuda_grouped_points.emplace_back(
            std::vector<QuadBoundaryPoint>{current});
      } else {
        cuda_grouped_points.back().emplace_back(current);
      }
      start = current;
    }

    return cuda_grouped_points;
  }

  // Groups marker pairs by runs of rep01().
  std::vector<std::vector<IndexPoint>> CudaGroupedPoints(
      const std::vector<IndexPoint> &compressed_cuda_points) const {
    std::vector<std::vector<IndexPoint>> cuda_grouped_points;
    cuda_grouped_points.emplace_back();

    IndexPoint start = compressed_cuda_points[0];
    for (size_t i = 0; i < compressed_cuda_points.size(); ++i) {
      IndexPoint current = compressed_cuda_points[i];
      CHECK_GE(current.blob_index(), start.blob_index())
          << " Failed on index " << i << " of "
          << compressed_cuda_points.size();
      if ((start.blob_index()) != (current.blob_index())) {
        cuda_grouped_points.emplace_back(std::vector<IndexPoint>{current});
      } else {
        cuda_grouped_points.back().emplace_back(current);
      }
      start = current;
    }

    return cuda_grouped_points;
  }

  // Checks that both april robotics and Cuda agree on point grouping.  No
  // ordering is implied here, that is checked later.
  void CheckAprilRoboticsAndCudaContainSamePoints(
      const std::vector<std::vector<uint64_t>> &april_grouped_points,
      const std::vector<std::vector<QuadBoundaryPoint>> &cuda_grouped_points)
      const {
    const std::vector<std::vector<QuadBoundaryPoint>>
        cuda_point_sorted_grouped_points = SortPoints(cuda_grouped_points);
    const std::vector<std::vector<uint64_t>> april_sorted_grouped_points =
        SortAprilroboticsPoints(april_grouped_points);

    CHECK_EQ(april_sorted_grouped_points.size(),
             cuda_point_sorted_grouped_points.size());
    for (size_t i = 0; i < april_sorted_grouped_points.size(); ++i) {
      CHECK_EQ(april_sorted_grouped_points[i].size(),
               cuda_point_sorted_grouped_points[i].size());
      for (size_t j = 0; j < april_sorted_grouped_points[i].size(); ++j) {
        CHECK_EQ(april_sorted_grouped_points[i][j],
                 cuda_point_sorted_grouped_points[i][j].x() +
                     cuda_point_sorted_grouped_points[i][j].y() * width_)
            << ": On list of size " << april_sorted_grouped_points[i].size()
            << ", failed on point " << j << "("
            << april_sorted_grouped_points[i][j] % width_ << ", "
            << april_sorted_grouped_points[i][j] / width_ << ") vs ("
            << cuda_point_sorted_grouped_points[i][j].x() << ", "
            << cuda_point_sorted_grouped_points[i][j].y() << ")";
      }
    }
  }

  // Tests that the extents and dot products match the cuda versions.
  std::pair<size_t, std::vector<IndexPoint>> CheckCudaExtents(
      const std::vector<std::vector<QuadBoundaryPoint>> &cuda_grouped_points,
      const std::vector<MinMaxExtents> &extents_cuda) const {
    std::vector<IndexPoint> selected_blobs;
    const size_t max_april_tag_perimeter = 2 * (width_ + height_);
    size_t selected_quads = 0;
    {
      BlobExtentsIndexFinder finder(extents_cuda.data(), extents_cuda.size());
      size_t i = 0;
      size_t starting_offset = 0;
      CHECK_EQ(cuda_grouped_points.size(), extents_cuda.size());
      for (const std::vector<QuadBoundaryPoint> &points : cuda_grouped_points) {
        size_t min_x, min_y, max_x, max_y;
        min_x = max_x = points[0].x();
        min_y = max_y = points[0].y();

        for (const QuadBoundaryPoint pt : points) {
          size_t x = pt.x();
          size_t y = pt.y();
          min_x = std::min(min_x, x);
          max_x = std::max(max_x, x);
          min_y = std::min(min_y, y);
          max_y = std::max(max_y, y);
        }

        const MinMaxExtents extents = extents_cuda[i];
        CHECK_EQ(extents.min_x, min_x);
        CHECK_EQ(extents.max_x, max_x);
        CHECK_EQ(extents.min_y, min_y);
        CHECK_EQ(extents.max_y, max_y);
        CHECK_EQ(extents.count, points.size());
        CHECK_EQ(extents.starting_offset, starting_offset)
            << " for index " << i;

        float dot = 0;

        const float cx = (min_x + max_x) * 0.5 + 0.05118;
        const float cy = (min_y + max_y) * 0.5 - 0.028581;

        bool good_blob_size =
            points.size() >= 24 && points.size() <= max_april_tag_perimeter &&
            points.size() >=
                static_cast<size_t>(tag_detector_->qtp.min_cluster_pixels) &&
            static_cast<int>((max_x - min_x) * (max_y - min_y)) >=
                min_tag_width_;

        for (size_t j = 0; j < points.size(); ++j) {
          dot += (static_cast<float>(points[j].x()) - cx) * points[j].gx() +
                 (static_cast<float>(points[j].y()) - cy) * points[j].gy();

          // Make sure our blob finder agrees.
          CHECK_EQ(i, finder.FindBlobIndex(starting_offset + j));
        }

        // Test that the summed dot product is right.
        CHECK_LT(std::abs(extents.dot() - dot) / std::abs(dot), 1e-2)
            << ": for point " << i << ", cuda -> " << extents.dot()
            << ", C++ -> " << dot;

        const bool quad_reversed_border = dot < 0;

        VLOG(1) << "For point " << i << ", cuda -> " << extents.dot()
                << ", C++ -> " << dot << " size " << points.size() << " border "
                << (!(!reversed_border_ && quad_reversed_border) &&
                    !(!normal_border_ && !quad_reversed_border))
                << " area: "
                << (extents.max_x - extents.min_x) *
                       (extents.max_y - extents.min_y)
                << " min_area: " << min_tag_width_;

        if (good_blob_size && !(!reversed_border_ && quad_reversed_border) &&
            !(!normal_border_ && !quad_reversed_border)) {
          ++selected_quads;
          for (size_t j = 0; j < points.size(); ++j) {
            IndexPoint pt(i, points[j].point_bits());
            float theta =
                (atan2f(pt.y() - extents.cy(), pt.x() - extents.cx()) + M_PI) *
                8e6;
            long long int theta_int = llrintf(theta);

            pt.set_theta(theta_int);
            selected_blobs.emplace_back(pt);
          }
        }

        starting_offset += points.size();
        ++i;
      }
    }
    return std::make_pair(selected_quads, selected_blobs);
  }

  // Tests that the filtered points lists are sorted by key.
  void CheckFilteredCudaPoints(
      const std::vector<IndexPoint> &selected_blobs,
      const std::vector<IndexPoint> &selected_blobs_cuda) const {
    CHECK_EQ(selected_blobs.size(), selected_blobs_cuda.size());
    for (size_t i = 0;
         i < std::min(selected_blobs.size(), selected_blobs_cuda.size()); ++i) {
      VLOG(1) << "Got blob[" << i << "] -> " << selected_blobs[i] << " vs "
              << selected_blobs_cuda[i];
      CHECK_EQ(selected_blobs[i].blob_index(),
               selected_blobs_cuda[i].blob_index());
    }
  }

  // Tests that the cuda sorted point list matches C++.
  void CheckSortedFilteredCudaPoints(
      const std::vector<std::vector<QuadBoundaryPoint>>
          &slope_sorted_expected_grouped_points,
      size_t selected_quads, const std::vector<IndexPoint> &selected_blobs,
      const std::vector<IndexPoint> &sorted_selected_blobs_cuda) const {
    CHECK_EQ(selected_blobs.size(), sorted_selected_blobs_cuda.size());
    const std::vector<std::vector<IndexPoint>> cuda_grouped_points =
        CudaGroupedPoints(sorted_selected_blobs_cuda);

    CHECK_EQ(cuda_grouped_points.size(), selected_quads);
    CHECK_EQ(slope_sorted_expected_grouped_points.size(), selected_quads);

    size_t missmatched_points = 0;
    for (size_t i = 0; i < cuda_grouped_points.size(); ++i) {
      const std::vector<IndexPoint> &cuda_grouped_blob = cuda_grouped_points[i];
      const std::vector<QuadBoundaryPoint> &slope_sorted_points =
          slope_sorted_expected_grouped_points[i];

      CHECK_EQ(cuda_grouped_blob.size(), slope_sorted_points.size());
      if (VLOG_IS_ON(1) && cuda_grouped_blob[0].blob_index() == 160) {
        for (size_t j = 0; j < cuda_grouped_points[i].size(); ++j) {
          LOG(INFO) << "For blob " << cuda_grouped_blob[0].blob_index()
                    << ", got " << cuda_grouped_blob[j] << " ("
                    << cuda_grouped_blob[j].x() << ", "
                    << cuda_grouped_blob[j].y() << ") expected "
                    << slope_sorted_points[j] << " ("
                    << slope_sorted_points[j].x() << ", "
                    << slope_sorted_points[j].y() << ")";
        }
      }

      size_t missmatched_runs = 0;

      // recalculates the theta to be used in the following check
      std::map<uint64_t, std::pair<float, float>> blob_centers;
      auto ComputeTheta =
          [&blob_centers](QuadBoundaryPoint pair) -> long double {
        const int32_t x = pair.x();
        const int32_t y = pair.y();

        auto blob_center = blob_centers.find(pair.rep01());
        CHECK(blob_center != blob_centers.end());

        const float cx = blob_center->second.first;
        const float cy = blob_center->second.second;

        float dx = x - cx;
        float dy = y - cy;

        return atan2f64(dy, dx);
      };

      // refinds blob centers
      for (size_t i = 0; i < slope_sorted_points.size();) {
        uint64_t first_rep01 = slope_sorted_points[i].rep01();

        int min_x, min_y, max_x, max_y;
        min_x = max_x = slope_sorted_points[i].x();
        min_y = max_y = slope_sorted_points[i].y();

        size_t j = i;
        for (; j < slope_sorted_points.size() &&
               slope_sorted_points[j].rep01() == first_rep01;
             ++j) {
          QuadBoundaryPoint pt = slope_sorted_points[j];

          int x = pt.x();
          int y = pt.y();
          min_x = std::min(min_x, x);
          max_x = std::max(max_x, x);
          min_y = std::min(min_y, y);
          max_y = std::max(max_y, y);
        }

        const float cx = (max_x + min_x) * 0.5 + 0.05118;
        const float cy = (max_y + min_y) * 0.5 + -0.028581;

        blob_centers[first_rep01] = std::make_pair(cx, cy);
        i = j;
      }

      for (size_t j = 0; j < cuda_grouped_points[i].size(); ++j) {
        if (cuda_grouped_blob[j].x() != slope_sorted_points[j].x() ||
            cuda_grouped_blob[j].y() != slope_sorted_points[j].y()) {
          size_t allowable_swapped_indices = 1;
          long double max_allowed_imprecision = 3e-7;
          // search through indixes j - a to j + a to see if they're swapped
          // also only doesn't count if the precision needed to differentiate is
          // less than max_allowed_imprecision
          bool is_allowable = false;
          for (size_t k = j - allowable_swapped_indices;
               k <= j + allowable_swapped_indices; k++) {
            if (cuda_grouped_blob[j].x() == slope_sorted_points[k].x() &&
                cuda_grouped_blob[j].y() == slope_sorted_points[k].y() &&
                abs(ComputeTheta(slope_sorted_points[k]) -
                    ComputeTheta(slope_sorted_points[j])) <
                    max_allowed_imprecision) {
              is_allowable = true;
            }
          }
          if (is_allowable) {
            continue;
          }
          ++missmatched_points;
          ++missmatched_runs;
          // We shouldn't see a lot of points in a row which don't match.
          VLOG(0) << "Missmatched point in blob "
                  << cuda_grouped_blob[0].blob_index() << ", point " << j
                  << " (" << cuda_grouped_blob[j].x() << ", "
                  << cuda_grouped_blob[j].y() << "), theta "
                  << cuda_grouped_blob[j].theta() << " vs ("
                  << slope_sorted_points[j].x() << ", "
                  << slope_sorted_points[j].y() << "), in size "
                  << cuda_grouped_points[i].size();
          CHECK_LE(missmatched_runs, 4u);
        } else {
          missmatched_runs = 0;
        }
      }
    }

    // Or a lot of points overall.  The slope algo has duplicate points, and
    // is occasionally wrong.
    CHECK_LE(missmatched_points, 25u);
  }

  void CheckCudaFilteredExtents(
      const std::vector<cub::KeyValuePair<long, MinMaxExtents>>
          &selected_extents_cuda,
      const std::vector<IndexPoint> &sorted_selected_blobs_cuda) {
    size_t start = 0;
    size_t sorted_point_index = 0;
    for (size_t i = 0; i < selected_extents_cuda.size(); ++i) {
      VLOG(1) << "Extent " << i << " started at "
              << selected_extents_cuda[i].value.starting_offset
              << " with count " << selected_extents_cuda[i].value.count;
      CHECK_EQ(selected_extents_cuda[i].value.starting_offset, start)
          << " for extent " << i;
      size_t found_blobs = 0;
      CHECK_EQ(selected_extents_cuda[i].value.starting_offset,
               sorted_point_index);
      while (sorted_selected_blobs_cuda[sorted_point_index].blob_index() == i) {
        ++found_blobs;
        ++sorted_point_index;
      }
      CHECK_EQ(found_blobs, selected_extents_cuda[i].value.count);
      start += selected_extents_cuda[i].value.count;
    }
  }

  void CheckErrors(const std::vector<IndexPoint> &sorted_selected_blobs_cuda,
                   const std::vector<LineFitPoint> &line_fit_points_cuda,
                   const std::vector<double> &errors_device,
                   const std::vector<double> &filtered_errors_device,
                   const std::vector<Peak> &peaks_device) {
    std::vector<std::vector<IndexPoint>> grouped_points =
        CudaGroupedPoints(sorted_selected_blobs_cuda);
    image_u8_t quad_im = ToImageu8t(decimated_cuda_);
    size_t accumulated_size = 0u;
    size_t blob_index = 0u;
    size_t bad_errors = 0u;
    size_t summed_cuda_pts = 0;
    for (const std::vector<IndexPoint> &group : grouped_points) {
      std::vector<double> errs;
      errs.resize(group.size(), 0.0);
      const size_t ksz = std::min<size_t>(20u, group.size() / 12);

      zarray_t *cluster = zarray_create(sizeof(struct pt));

      for (size_t i = 0; i < group.size(); i++) {
        pt p{
            .x = static_cast<uint16_t>(group[i].x()),
            .y = static_cast<uint16_t>(group[i].y()),
        };
        zarray_add(cluster, &p);
      }

      CHECK_EQ(static_cast<size_t>(zarray_size(cluster)), group.size());

      struct line_fit_pt *lfps = compute_lfps(group.size(), cluster, &quad_im);

      VLOG(1) << "Inspecting blob of size " << group.size() << " global start "
              << accumulated_size;
      for (size_t i = 0; i < group.size(); i++) {
        if (group[i].blob_index() ==
            (size_t)absl::GetFlag(FLAGS_debug_blob_index)) {
          LOG(INFO) << "For idx " << i << " global " << accumulated_size + i
                    << "(" << group[i].x() << ", " << group[i].y()
                    << "), cuda: " << line_fit_points_cuda[accumulated_size + i]
                    << " aprilrobotics " << lfps[i];
        }
        CHECK_EQ(line_fit_points_cuda[accumulated_size + i].Mx / 2.0,
                 lfps[i].Mx)
            << ": for blob index " << group[i].blob_index();
        CHECK_EQ(line_fit_points_cuda[accumulated_size + i].My / 2.0,
                 lfps[i].My)
            << ": for blob index " << group[i].blob_index();
        CHECK_EQ(line_fit_points_cuda[accumulated_size + i].Mxx / 4.0,
                 lfps[i].Mxx)
            << ": for blob index " << group[i].blob_index();
        CHECK_EQ(line_fit_points_cuda[accumulated_size + i].Mxy / 4.0,
                 lfps[i].Mxy)
            << ": for blob index " << group[i].blob_index();
        CHECK_EQ(line_fit_points_cuda[accumulated_size + i].Myy / 4.0,
                 lfps[i].Myy)
            << ": for blob index " << group[i].blob_index();
        CHECK_EQ(line_fit_points_cuda[accumulated_size + i].W, lfps[i].W)
            << ": for blob index " << group[i].blob_index();
      }

      for (size_t i = 0; i < group.size(); i++) {
        if (group[i].blob_index() ==
            (size_t)absl::GetFlag(FLAGS_debug_blob_index)) {
          LOG(INFO) << "  Cuda error[" << i << "] -> "
                    << errors_device[accumulated_size + i] << ", filtered "
                    << filtered_errors_device[i + accumulated_size];
        }
      }

      for (size_t i = 0; i < group.size(); i++) {
        const size_t i0 = (i + group.size() - ksz) % group.size();
        const size_t i1 = (i + ksz) % group.size();
        const size_t gi0 = accumulated_size + i0;
        const size_t gi1 = accumulated_size + i1;

        fit_line(lfps, group.size(), i0, i1, NULL, &errs[i], NULL);

        if (std::abs(errs[i] - errors_device[accumulated_size + i]) >=
            0.001 * std::max<double>(1.0, errs[i])) {
          VLOG(1) << "i0   aprilrobotics " << lfps[i0];
          VLOG(1) << "i0            cuda " << line_fit_points_cuda[gi0];
          VLOG(1) << " dMx: "
                  << (lfps[i0].Mx * 2 - line_fit_points_cuda[gi0].Mx);
          VLOG(1) << " dMy: "
                  << (lfps[i0].My * 2 - line_fit_points_cuda[gi0].My);
          VLOG(1) << " dMxx: "
                  << (lfps[i0].Mxx * 4 - line_fit_points_cuda[gi0].Mxx);
          VLOG(1) << " dMxy: "
                  << (lfps[i0].Mxy * 4 - line_fit_points_cuda[gi0].Mxy);
          VLOG(1) << " dMyy: "
                  << (lfps[i0].Myy * 4 - line_fit_points_cuda[gi0].Myy);
          VLOG(1) << " dW: " << (lfps[i0].W - line_fit_points_cuda[gi0].W);
          VLOG(1) << "i1   aprilrobotics " << lfps[i1];
          VLOG(1) << "i1            cuda " << line_fit_points_cuda[gi1];
          VLOG(1) << " dMx: "
                  << (lfps[i1].Mx * 2 - line_fit_points_cuda[gi1].Mx);
          VLOG(1) << " dMy: "
                  << (lfps[i1].My * 2 - line_fit_points_cuda[gi1].My);
          VLOG(1) << " dMxx: "
                  << (lfps[i1].Mxx * 4 - line_fit_points_cuda[gi1].Mxx);
          VLOG(1) << " dMxy: "
                  << (lfps[i1].Mxy * 4 - line_fit_points_cuda[gi1].Mxy);
          VLOG(1) << " dMyy: "
                  << (lfps[i1].Myy * 4 - line_fit_points_cuda[gi1].Myy);
          VLOG(1) << " dW: " << (lfps[i1].W - line_fit_points_cuda[gi1].W);
          VLOG(1) << "derror: " << errs[i] << " - "
                  << errors_device[accumulated_size + i] << " = "
                  << errs[i] - errors_device[accumulated_size + i];

          ++bad_errors;
        }
      }

      std::vector<float> filter_coefficients;
      {
        // Stolen directly from aprilrobotics to test.

        double sigma = 1;  // was 3

        // cutoff = exp(-j*j/(2*sigma*sigma));
        // log(cutoff) = -j*j / (2*sigma*sigma)
        // log(cutoff)*2*sigma*sigma = -j*j;

        // how big a filter should we use? We make our kernel big
        // enough such that we represent any values larger than
        // 'cutoff'.

        // XXX Tunable (though not super useful to change)
        double cutoff = 0.05;
        int fsz = sqrt(-log(cutoff) * 2 * sigma * sigma) + 1;
        fsz = 2 * fsz + 1;

        // For default values of cutoff = 0.05, sigma = 3,
        // we have fsz = 17.

        for (int i = 0; i < fsz; i++) {
          int j = i - fsz / 2;
          filter_coefficients.push_back(exp(-j * j / (2 * sigma * sigma)));
          CHECK_EQ(filter_coefficients[i], FilterCoefficients()[i]);
        }
      }

      std::vector<double> filtered_errors;
      filtered_errors.resize(group.size(), 0.0);

      for (size_t i = 0; i < group.size(); i++) {
        double accumulated_cuda = 0;
        double accumulated = 0;
        for (int filter_coefficient_index = 0;
             filter_coefficient_index <
             static_cast<int>(filter_coefficients.size());
             filter_coefficient_index++) {
          const double e =
              errs[(i + filter_coefficient_index -
                    filter_coefficients.size() / 2 + group.size()) %
                   group.size()];
          accumulated += e * filter_coefficients[filter_coefficient_index];

          const double e_cuda =
              errors_device[(i + filter_coefficient_index -
                             filter_coefficients.size() / 2 + group.size()) %
                                group.size() +
                            accumulated_size];
          accumulated_cuda +=
              e_cuda * filter_coefficients[filter_coefficient_index];
        }
        filtered_errors[i] = accumulated;

        // Make sure the filter math works.  This can be a lot tighter since the
        // error calc appears to be less accurate.
        CHECK_LT(std::abs(accumulated_cuda -
                          filtered_errors_device[accumulated_size + i]),
                 1e-3)
            << " Failed to match error at blob " << blob_index << " index " << i
            << " global index " << accumulated_size + i
            << " for a blob of size " << group.size() << " expected "
            << filtered_errors[i] << " got "
            << filtered_errors_device[accumulated_size + i];

        // Make sure it hasn't walked too far off from the actual error.
        /*
        CHECK_LT(std::abs(filtered_errors[i] -
                          filtered_errors_device[accumulated_size + i]),
                 1e-1)
            << " Failed to match error at blob " << blob_index << " index " << i
            << " global index " << accumulated_size + i
            << " for a blob of size " << group.size() << " expected "
            << filtered_errors[i] << " got "
            << filtered_errors_device[accumulated_size + i];
            */
      }

      // Now, check the peaks.
      size_t num_peaks_cuda = 0;
      size_t num_peaks_april = 0;
      for (size_t i = 0; i < group.size(); i++) {
        double before_aprilrobotics =
            filtered_errors[(i + group.size() - 1) % group.size()];
        double after_aprilrobotics = filtered_errors[(i + 1) % group.size()];
        double us_aprilrobotics = filtered_errors[i];

        double before_cuda =
            filtered_errors_device[(i + group.size() - 1) % group.size() +
                                   accumulated_size];
        double after_cuda =
            filtered_errors_device[(i + 1) % group.size() + accumulated_size];
        double us_cuda = filtered_errors_device[i + accumulated_size];

        const bool is_peak_aprilrobotics =
            us_aprilrobotics > before_aprilrobotics &&
            us_aprilrobotics > after_aprilrobotics;
        const bool is_peak_cuda = us_cuda > before_cuda && us_cuda > after_cuda;

        if (is_peak_aprilrobotics) {
          ++num_peaks_april;
        }
        // We don't agree...  But it is on little stupid stuff.
        /*CHECK_EQ(is_peak_aprilrobotics, peaks_device[accumulated_size + i])
            << " Failed to match peak at blob " << blob_index << " index " << i
            << " global index " << accumulated_size + i
            << " for a blob of size " << group.size() << " compared "
            << before_aprilrobotics << " vs " << us_aprilrobotics << " vs "
            << after_aprilrobotics;*/
        CHECK_EQ(is_peak_cuda, peaks_device[accumulated_size + i].blob_index !=
                                   Peak::kNoPeak())
            << " Failed to match peak at blob " << blob_index << " index " << i
            << " global index " << accumulated_size + i
            << " for a blob of size " << group.size() << " compared "
            << before_cuda << " vs " << us_cuda << " vs " << after_cuda;
        CHECK_EQ(peaks_device[accumulated_size + i].filtered_point_index,
                 accumulated_size + i);
        CHECK_LE(peaks_device[accumulated_size + i].error,
                 -filtered_errors_device[accumulated_size + i] + 1e-3);
        CHECK_GE(peaks_device[accumulated_size + i].error,
                 -filtered_errors_device[accumulated_size + i] - 1e-3);
        if (is_peak_cuda) {
          CHECK_EQ(peaks_device[accumulated_size + i].blob_index,
                   group[0].blob_index())
              << " Failed to match peak at blob " << blob_index << " index "
              << i << " global index " << accumulated_size + i
              << " for a blob of size " << group.size() << " compared "
              << before_cuda << " vs " << us_cuda << " vs " << after_cuda;
        }
        if (is_peak_cuda) {
          ++num_peaks_cuda;
          ++summed_cuda_pts;
        }
      }

      (void)num_peaks_cuda;
      (void)num_peaks_april;
      // CHECK_EQ(num_peaks_cuda, num_peaks_april);

      zarray_destroy(cluster);
      free(lfps);
      accumulated_size += group.size();
      ++blob_index;
    }
    LOG(INFO) << "Overall, found " << summed_cuda_pts << " peaks with "
              << bad_errors << " bad errors.";
    // CHECK_LT(bad_errors, 60u);
  }

  // Orders the clusters reported by AprilRobotics to match the inbound cuda
  // cluster order.
  zarray_t *OrderAprilroboticsLikeCuda(
      image_u8_t *thresholded_im, unionfind_t *uf,
      const std::vector<std::vector<QuadBoundaryPoint>> &cuda_grouped_points)
      const {
    // Step 1, label both the cuda and april robotics points with their original
    // index. Step 2, sort both, retaining the indices. Step 3, reorder the
    // april robotics points to match the cuda points using the indices.
    zarray_t *clusters =
        gradient_clusters(tag_detector_, thresholded_im, thresholded_im->width,
                          thresholded_im->height, thresholded_im->stride, uf);

    // Reorder a copy of april robotics clusters to our format.
    struct AprilClusterIndexPair {
      size_t index;
      std::vector<uint64_t> points;
    };
    std::vector<AprilClusterIndexPair> april_points;

    for (int i = 0; i < zarray_size(clusters); i++) {
      zarray_t *cluster;
      zarray_get(clusters, i, &cluster);

      AprilClusterIndexPair points;
      points.index = i;

      for (int j = 0; j < zarray_size(cluster); j++) {
        struct pt *p;
        zarray_get_volatile(cluster, j, &p);

        points.points.push_back(p->x + p->y * width_);
      }

      std::sort(points.points.begin(), points.points.end());
      points.points.erase(
          std::unique(points.points.begin(), points.points.end()),
          points.points.end());
      april_points.emplace_back(std::move(points));
    }

    // And sort the blobs now.
    std::sort(april_points.begin(), april_points.end(), [](auto &x, auto &y) {
      if (x.points.size() == y.points.size()) {
        for (size_t j = 0; j < x.points.size(); ++j) {
          if (x.points[j] == y.points[j]) {
            continue;
          }
          return x.points[j] < y.points[j];
        }
        LOG(FATAL) << "Equal";
      }
      return x.points.size() < y.points.size();
    });

    struct CudaClusterIndexPair {
      size_t index;
      std::vector<QuadBoundaryPoint> points;
    };

    // Do the same for CUDA.
    std::vector<CudaClusterIndexPair> cuda_points;
    for (size_t i = 0; i < cuda_grouped_points.size(); ++i) {
      CudaClusterIndexPair points;
      points.points = cuda_grouped_points[i];
      points.index = i;

      std::sort(points.points.begin(), points.points.end(),
                [this](const QuadBoundaryPoint &a, const QuadBoundaryPoint &b) {
                  CHECK_EQ(a.rep01(), b.rep01());
                  return a.x() + a.y() * width_ < b.x() + b.y() * width_;
                });
      cuda_points.emplace_back(std::move(points));
    }

    // And sort the blobs.
    std::sort(cuda_points.begin(), cuda_points.end(), [this](auto &x, auto &y) {
      if (x.points.size() == y.points.size()) {
        for (size_t j = 0; j < x.points.size(); ++j) {
          if (x.points[j].x() + x.points[j].y() * width_ ==
              y.points[j].x() + y.points[j].y() * width_) {
            continue;
          }
          return x.points[j].x() + x.points[j].y() * width_ <
                 y.points[j].x() + y.points[j].y() * width_;
        }
        LOG(FATAL) << "Equal";
      }
      return x.points.size() < y.points.size();
    });

    // OK, we now have the cuda points in the same order as the aprilrobotics
    // points.  First CHECK that they match, just in case...
    std::vector<zarray_t *> sorted_result;
    sorted_result.resize(cuda_points.size());

    CHECK_EQ(april_points.size(), cuda_points.size());
    for (size_t i = 0; i < april_points.size(); ++i) {
      CHECK_EQ(april_points[i].points.size(), cuda_points[i].points.size());
      for (size_t j = 0; j < april_points[i].points.size(); ++j) {
        CHECK_EQ(april_points[i].points[j],
                 cuda_points[i].points[j].x() +
                     cuda_points[i].points[j].y() * width_)
            << ": " << i << " " << j;
      }

      // And, while we are iterating, make a vector with the clusters in the
      // right order.
      zarray_t *cluster;
      zarray_get(clusters, april_points[i].index, &cluster);
      sorted_result[cuda_points[i].index] = cluster;
    }

    // Finally, put it back in the clusters.
    for (size_t i = 0; i < april_points.size(); ++i) {
      zarray_set(clusters, i, &sorted_result[i], nullptr);
    }

    return clusters;
  }

  void CheckQuads(const zarray_t *clusters,
                  std::vector<IndexPoint> sorted_selected_blobs_cuda,
                  const std::vector<QuadCorners> &fit_quads) {
    std::vector<std::vector<IndexPoint>> sorted_blobs;
    for (auto blob : CudaGroupedPoints(sorted_selected_blobs_cuda)) {
      while (blob[0].blob_index() > sorted_blobs.size()) {
        sorted_blobs.emplace_back();
      }
      sorted_blobs.emplace_back(std::move(blob));
    }

    image_u8_t quad_im = ToImageu8t(decimated_cuda_);

    std::vector<bool> aprilrobotics_valid;
    std::vector<struct quad> aprilrobotics_quad;
    aprilrobotics_valid.resize(zarray_size(clusters));
    aprilrobotics_quad.resize(zarray_size(clusters));

    for (int i = 0; i < zarray_size(clusters); ++i) {
      struct quad quad_result;

      zarray_t *cluster;
      zarray_get(clusters, i, &cluster);

      if (i == absl::GetFlag(FLAGS_debug_blob_index)) {
        LOG(INFO) << "cuda points for blob " << i << " are";
        for (size_t j = 0; j < sorted_blobs[i].size(); ++j) {
          LOG(INFO) << "  blob[" << j << "]: (" << sorted_blobs[i][j].x()
                    << ", " << sorted_blobs[i][j].y() << ")";
        }
      }
      VLOG(1) << "Going to fit quad " << i << " of size "
              << zarray_size(cluster);
      int valid_blob =
          fit_quad(tag_detector_, &quad_im, cluster, &quad_result,
                   min_tag_width_, normal_border_, reversed_border_);

      auto quad_iterator = std::find_if(fit_quads.begin(), fit_quads.end(),
                                        [&](const QuadCorners corner) {
                                          return (int)corner.blob_index == i;
                                        });

      CHECK_EQ(quad_iterator != fit_quads.end(), valid_blob != 0)
          << ": Missmatch on quad " << i;

      if (!valid_blob) {
        continue;
      }

      // Apply the center adjustment algorithm to the ground truth so we compare
      // apples to apples.
      gpu_detector_.AdjustCenter(quad_result.p);

      QuadCorners cuda_corner = *quad_iterator;

      for (size_t point = 0; point < 4; ++point) {
        constexpr double kEpsilon = 1e-3;
        CHECK_LE(quad_result.p[point][0],
                 cuda_corner.corners[point][0] + kEpsilon);
        CHECK_GE(quad_result.p[point][0],
                 cuda_corner.corners[point][0] - kEpsilon);
        CHECK_LE(quad_result.p[point][1],
                 cuda_corner.corners[point][1] + kEpsilon);
        CHECK_GE(quad_result.p[point][1],
                 cuda_corner.corners[point][1] - kEpsilon);
      }
    }
  }

  static inline int DetectionCompareFunction(const void *_a, const void *_b) {
    apriltag_detection_t *a = *(apriltag_detection_t **)_a;
    apriltag_detection_t *b = *(apriltag_detection_t **)_b;

    if (a->id != b->id) {
      return a->id - b->id;
    } else if (a->hamming != b->hamming) {
      return a->hamming - b->hamming;
    } else {
      return b->decision_margin - a->decision_margin;
    }
  }

  void set_undistort(bool value) { undistort_ = value; }

  void CheckDetections(zarray_t *aprilrobotics_detections,
                       const zarray_t *_gpu_detections) {
    zarray_t *gpu_detections = zarray_copy(_gpu_detections);
    zarray_sort(gpu_detections, DetectionCompareFunction);
    zarray_sort(aprilrobotics_detections, DetectionCompareFunction);

    CHECK_EQ(zarray_size(aprilrobotics_detections),
             zarray_size(gpu_detections));
    LOG(INFO) << "Found " << zarray_size(gpu_detections) << " tags";

    for (int i = 0; i < zarray_size(aprilrobotics_detections); ++i) {
      const apriltag_detection_t *aprilrobotics_detection;
      const apriltag_detection_t *gpu_detection;

      zarray_get(aprilrobotics_detections, i, &aprilrobotics_detection);
      zarray_get(gpu_detections, i, &gpu_detection);

      bool valid = gpu_detection->decision_margin >
                   absl::GetFlag(FLAGS_min_decision_margin);

      LOG(INFO) << "Found GPU " << (valid ? "valid" : "invalid")
                << " tag number " << gpu_detection->id
                << " hamming: " << gpu_detection->hamming
                << " margin: " << gpu_detection->decision_margin;
      LOG(INFO) << "Found CPU " << (valid ? "valid" : "invalid")
                << " tag number " << aprilrobotics_detection->id
                << " hamming: " << aprilrobotics_detection->hamming
                << " margin: " << aprilrobotics_detection->decision_margin;
    }

    for (int i = 0; i < zarray_size(aprilrobotics_detections); ++i) {
      const apriltag_detection_t *aprilrobotics_detection;
      const apriltag_detection_t *gpu_detection;

      zarray_get(aprilrobotics_detections, i, &aprilrobotics_detection);
      zarray_get(gpu_detections, i, &gpu_detection);

      const bool valid = gpu_detection->decision_margin >
                         absl::GetFlag(FLAGS_min_decision_margin);

      // TODO(austin): Crank down the thresholds and figure out why these
      // deviate.  It should be the same function for both at this point.
      const double threshold = undistort_ ? 15.0 : (valid ? 2e-3 : 1e-1);

      CHECK_EQ(aprilrobotics_detection->id, gpu_detection->id);
      CHECK_EQ(aprilrobotics_detection->hamming, gpu_detection->hamming);
      EXPECT_NEAR(aprilrobotics_detection->c[0], gpu_detection->c[0],
                  threshold);
      EXPECT_NEAR(aprilrobotics_detection->c[1], gpu_detection->c[1],
                  threshold);

      for (int j = 0; j < 4; ++j) {
        for (int k = 0; k < 2; ++k) {
          EXPECT_NEAR(aprilrobotics_detection->p[j][k], gpu_detection->p[j][k],
                      threshold);
        }
      }

      CHECK_EQ(aprilrobotics_detection->H->nrows, gpu_detection->H->nrows);
      CHECK_EQ(aprilrobotics_detection->H->ncols, gpu_detection->H->ncols);

      for (size_t j = 0; j < gpu_detection->H->ncols; ++j) {
        for (size_t k = 0; k < gpu_detection->H->nrows; ++k) {
          EXPECT_NEAR(matd_get(gpu_detection->H, k, j),
                      matd_get(aprilrobotics_detection->H, k, j), threshold);
        }
      }
    }
  }

  // Checks that the GPU and CPU algorithms match.
  void Check(cv::Mat color_image) {
    cv::Mat gray_image(color_image.size(), CV_8UC1);
    cv::cvtColor(color_image, gray_image, cv::COLOR_YUV2GRAY_YUYV);

    image_u8_t gray_im = ToImageu8t(gray_image);
    CheckImage(gray_im, ToImageu8t(gray_cuda_), "gray_cuda");

    image_u8_t *quad_im =
        image_u8_decimate(&gray_im, tag_detector_->quad_decimate);
    CheckImage(*quad_im, ToImageu8t(decimated_cuda_), "decimated_cuda");

    image_u8_t *thresholded_im = threshold(tag_detector_, quad_im);
    CheckImage(ToImageu8t(thresholded_cuda_), *thresholded_im,
               "threshold_cuda");

    unionfind_t *uf = connected_components(
        tag_detector_, thresholded_im, thresholded_im->width,
        thresholded_im->height, thresholded_im->stride);

    // Checks union finding.
    auto blob_sizes = CheckUnionfind(
        uf, union_markers_,
        reinterpret_cast<const uint32_t *>(union_markers_size_.data),
        thresholded_cuda_);

    // Now make sure our unfiltered and partially sorted lists of pairs are
    // plausible.
    const std::vector<QuadBoundaryPoint> expected_union_marker_pair =
        CheckUnionMarkerPairs(blob_sizes);
    const std::vector<QuadBoundaryPoint> expected_compressed_union_marker_pair =
        CheckCompressedUnionMarkerPairs(expected_union_marker_pair,
                                        compressed_union_marker_pair_,
                                        sorted_union_marker_pair_);

    const std::vector<std::vector<QuadBoundaryPoint>> cuda_grouped_points =
        CudaGroupedPoints(sorted_union_marker_pair_);
    PrintCudaPoints(cuda_grouped_points, blob_sizes);

    const std::vector<std::vector<QuadBoundaryPoint>>
        slope_sorted_expected_grouped_points = FilterBlobs(CudaGroupedPoints(
            SlopeSortPoints(expected_compressed_union_marker_pair)));

    const std::vector<std::vector<uint64_t>> april_grouped_points =
        AprilRoboticsPoints(thresholded_im, uf);

    LOG(INFO) << "Found " << april_grouped_points.size()
              << " clusters with april robotics with "
              << std::accumulate(
                     april_grouped_points.begin(), april_grouped_points.end(),
                     0, [](int size, const auto &v) { return size + v.size(); })
              << " points.";
    LOG(INFO) << "Found " << cuda_grouped_points.size()
              << " clusters with cuda with "
              << num_compressed_union_marker_pair_ << " points.";

    // Verify that both aprilrobotics and us group points the same.  Ignore
    // order.
    CheckAprilRoboticsAndCudaContainSamePoints(april_grouped_points,
                                               cuda_grouped_points);

    // Test that the extents are reasonable.
    size_t selected_quads;
    std::vector<IndexPoint> selected_blobs;
    std::tie(selected_quads, selected_blobs) =
        CheckCudaExtents(cuda_grouped_points, extents_cuda_);

    // And that the filtered list is reasonable.
    CheckFilteredCudaPoints(selected_blobs, selected_blobs_cuda_);

    // And that they sorted right too.
    CheckSortedFilteredCudaPoints(slope_sorted_expected_grouped_points,
                                  selected_quads, selected_blobs,
                                  sorted_selected_blobs_cuda_);

    CheckCudaFilteredExtents(selected_extents_cuda_,
                             sorted_selected_blobs_cuda_);

    CheckErrors(sorted_selected_blobs_cuda_, line_fit_points_cuda_,
                errors_device_, filtered_errors_device_, peaks_device_);

    zarray_t *april_clusters =
        OrderAprilroboticsLikeCuda(thresholded_im, uf, cuda_grouped_points);

    if (!undistort_) {
      CheckQuads(april_clusters, sorted_selected_blobs_cuda_, fit_quads_);
    }

    const zarray_t *gpu_detections = gpu_detector_.Detections();
    CheckDetections(aprilrobotics_detections_, gpu_detections);

    LOG(INFO) << "Found slope sorted count: "
              << sorted_union_marker_pair_.size();

    for (int i = 0; i < zarray_size(april_clusters); i++) {
      zarray_t *cluster;
      zarray_get(april_clusters, i, &cluster);
      zarray_destroy(cluster);
    }

    zarray_destroy(april_clusters);
    unionfind_destroy(uf);
    image_u8_destroy(quad_im);
    image_u8_destroy(thresholded_im);
  }

  // Writes images out to /tmp for debugging.
  void WriteDebug(cv::Mat color_image) {
    cv::Mat bgr(color_image.size(), CV_8UC3);
    cv::cvtColor(color_image, bgr, cv::COLOR_YUV2BGR_YUYV);
    cv::imwrite("/tmp/debug_color_image.png", bgr);

    cv::imwrite("/tmp/debug_halide_grey_cuda.png", gray_cuda_);
    cv::imwrite("/tmp/debug_halide_grey_decimated.png", decimated_cuda_);
    cv::imwrite("/tmp/debug_cuda_thresholded.png", thresholded_cuda_);

    {
      image_u8_t thresholded_halide_im = ToImageu8t(thresholded_cuda_);

      unionfind_t *uf = connected_components(
          tag_detector_, &thresholded_halide_im, thresholded_halide_im.width,
          thresholded_halide_im.height, thresholded_halide_im.stride);

      std::map<uint32_t, uint32_t> april_to_cuda_id_remap;
      std::map<uint32_t, uint32_t> cuda_to_april_id_remap;

      std::map<uint32_t, cv::Vec3b> colors;

      // Seed the random number generator so we get the same images out each
      // time for easier comparison.
      srandom(0);

      // Write out the union find image.
      size_t color_count = 1;
      uint32_t max_color = 0;
      cv::Mat unionfind_image(union_markers_.size(), CV_8UC3);
      cv::Mat unionfind_image_common(union_markers_.size(), CV_8UC3);
      for (int y = 0; y < union_markers_.rows; ++y) {
        for (int x = 0; x < union_markers_.cols; ++x) {
          const uint32_t april_robotics_id =
              unionfind_get_representative(uf, y * width_ / 2 + x);
          const uint32_t index = union_markers_.at<uint32_t>(y, x);

          bool one_to_one_blob_match = false;

          {
            auto it = cuda_to_april_id_remap.find(index);
            if (it == cuda_to_april_id_remap.end()) {
              // Now, check we don't have a match the other way.
              auto it_back = april_to_cuda_id_remap.find(april_robotics_id);
              if (it_back == april_to_cuda_id_remap.end()) {
                one_to_one_blob_match = true;
                cuda_to_april_id_remap.emplace(index, april_robotics_id);
                april_to_cuda_id_remap.emplace(april_robotics_id, index);
              } else {
                one_to_one_blob_match = false;
              }
            } else {
              auto it_back = april_to_cuda_id_remap.find(april_robotics_id);
              if (it_back == april_to_cuda_id_remap.end()) {
                one_to_one_blob_match = false;
              } else {
                if (it->second == april_robotics_id &&
                    it_back->second == index) {
                  one_to_one_blob_match = true;
                }
              }
            }
          }

          cv::Vec3b color;

          auto it = colors.find(index);
          if (it == colors.end()) {
            max_color = std::max(max_color, index);
            ++color_count;
            VLOG(2) << "New color 0x" << std::hex << index;
            const int bias = 50;
            uint8_t r = bias + (random() % (200 - bias));
            uint8_t g = bias + (random() % (200 - bias));
            uint8_t b = bias + (random() % (200 - bias));

            color = cv::Vec3b(b, g, r);
            colors[index] = color;
          } else {
            color = it->second;
          }

          unionfind_image.at<cv::Vec3b>(y, x) = color;

          if (one_to_one_blob_match) {
            if (index == static_cast<uint32_t>(x + y * union_markers_.cols)) {
              color = cv::Vec3b(0, 0, 0);
            } else if (thresholded_cuda_.at<uint8_t>(y, x) == 0) {
              color = cv::Vec3b(0, 0, 0);
            } else {
              color = cv::Vec3b(255, 255, 255);
            }
          }

          CHECK(one_to_one_blob_match);

          unionfind_image_common.at<cv::Vec3b>(y, x) = color;
        }
      }
      LOG(INFO) << "Found " << color_count << " colors with a max index of "
                << max_color;
      cv::imwrite("/tmp/debug_cuda_segmentation.png", unionfind_image);
      cv::imwrite("/tmp/debug_cuda_segmentation_common.png",
                  unionfind_image_common);

      unionfind_destroy(uf);
    }

    {
      // And write out the marker pairs image too.
      std::map<uint64_t, cv::Vec3b> colors;

      cv::Mat sorted_marker_pair(cv::Size((width_), (height_)), CV_8UC3);
      sorted_marker_pair.setTo(cv::Scalar(0, 0, 0));

      for (size_t i = 0; i < sorted_union_marker_pair_.size();) {
        size_t blob_size = 0;
        for (size_t j = i; j < sorted_union_marker_pair_.size(); ++j) {
          if (sorted_union_marker_pair_[i].rep01() !=
              sorted_union_marker_pair_[j].rep01()) {
            break;
          }
          ++blob_size;
        }
        for (size_t j = i; j < sorted_union_marker_pair_.size(); ++j) {
          if (sorted_union_marker_pair_[i].rep01() !=
              sorted_union_marker_pair_[j].rep01()) {
            break;
          }
          QuadBoundaryPoint pair = sorted_union_marker_pair_[j];

          const size_t x = pair.x();
          const size_t y = pair.y();

          auto it = colors.find(pair.rep01());
          cv::Vec3b color;
          if (it == colors.end()) {
            VLOG(2) << "New color 0x" << std::hex << pair.rep01();
            const int bias = 50;
            uint8_t r = bias + (random() % (200 - bias));
            uint8_t g = bias + (random() % (200 - bias));
            uint8_t b = bias + (random() % (200 - bias));

            color = cv::Vec3b(b, g, r);
            colors[pair.rep01()] = color;
          } else {
            color = it->second;
          }
          sorted_marker_pair.at<cv::Vec3b>(y + 2, x + 2) = color;
        }
        i += blob_size;
      }
      cv::imwrite("/tmp/debug_cuda_marker_pairs.png", sorted_marker_pair);
    }
  }

  // Sets the camera constants for camera 24-04
  void SetCameraFourConstants() {
    gpu_detector_.SetCameraMatrix(
        CameraMatrix{642.80365, 718.017517, 642.83667, 555.022461});
    gpu_detector_.SetDistortionCoefficients(
        DistCoeffs{-0.239969, 0.055889, 0.000086, 0.000099, -0.005468});
  }

 private:
  apriltag_family_t *tag_family_;
  apriltag_detector_t *tag_detector_;

  cv::Mat gray_cuda_;
  cv::Mat decimated_cuda_;
  cv::Mat thresholded_cuda_;

  cv::Mat union_markers_;
  cv::Mat union_markers_size_;

  std::vector<QuadBoundaryPoint> union_marker_pair_;
  std::vector<QuadBoundaryPoint> compressed_union_marker_pair_;
  std::vector<QuadBoundaryPoint> sorted_union_marker_pair_;
  std::vector<uint32_t> quad_length_;
  std::vector<MinMaxExtents> extents_cuda_;
  std::vector<cub::KeyValuePair<long, MinMaxExtents>> selected_extents_cuda_;
  std::vector<IndexPoint> selected_blobs_cuda_;
  std::vector<IndexPoint> sorted_selected_blobs_cuda_;
  std::vector<LineFitPoint> line_fit_points_cuda_;

  std::vector<double> errors_device_;
  std::vector<double> filtered_errors_device_;
  std::vector<Peak> peaks_device_;
  int num_compressed_union_marker_pair_ = 0;
  int num_quads_ = 0;
  std::vector<QuadCorners> fit_quads_;

  GpuDetector<frc971::apriltag::InputFormat::YCbCr422> gpu_detector_;

  zarray_t *aprilrobotics_detections_ = nullptr;

  size_t width_;
  size_t height_;

  bool normal_border_ = false;
  bool reversed_border_ = false;

  bool undistort_ = false;
  int min_tag_width_ = 1000000;
};

class AprilDetectionTest : public ::testing::Test {
 public:
  aos::FlatbufferVector<frc971::vision::CameraImage> ReadImage(
      std::string_view path) {
    return aos::FileToFlatbuffer<frc971::vision::CameraImage>(
        "../" + std::string(path));
  }

  cv::Mat ToMat(const frc971::vision::CameraImage *image) {
    cv::Mat color_image(cv::Size(image->cols(), image->rows()), CV_8UC2,
                        (void *)image->data()->data());
    return color_image;
  }
};

TEST_F(AprilDetectionTest, ImageRepeat) {
  auto image = ReadImage("orin_image_apriltag/file/orin_image_apriltag.bfbs");

  LOG(INFO) << "Image is: " << image.message().cols() << " x "
            << image.message().rows();

  CudaAprilTagDetector cuda_detector(image.message().cols(),
                                     image.message().rows());

  const cv::Mat color_image = ToMat(&image.message());

  for (size_t i = 0; i < 4; ++i) {
    LOG(INFO) << "Attempt " << i;
    cuda_detector.DetectGPU(color_image.clone());
    cuda_detector.DetectCPU(color_image.clone());
  }
  cuda_detector.Check(color_image.clone());
  if (absl::GetFlag(FLAGS_debug)) {
    cuda_detector.WriteDebug(color_image);
  }
}

class SingleAprilDetectionTest
    : public ::testing::WithParamInterface<std::string_view>,
      public AprilDetectionTest {};

// Tests a single image.
TEST_P(SingleAprilDetectionTest, Image) {
  auto image = ReadImage(GetParam());

  LOG(INFO) << "Testing " << GetParam() << " with dimensions "
            << image.message().cols() << " x " << image.message().rows();

  CudaAprilTagDetector cuda_detector(image.message().cols(),
                                     image.message().rows());

  const cv::Mat color_image = ToMat(&image.message());

  cuda_detector.DetectGPU(color_image.clone());
  cuda_detector.DetectCPU(color_image.clone());
  cuda_detector.Check(color_image.clone());
  if (absl::GetFlag(FLAGS_debug)) {
    cuda_detector.WriteDebug(color_image);
  }
}

// Tests that both algorithms agree on a bunch of pixels.
INSTANTIATE_TEST_SUITE_P(
    CapturedImages, SingleAprilDetectionTest,
    ::testing::Values(
        "apriltag_test_bfbs_images/"
        "bfbs-capture-2013-01-18_08-54-16.869057537.bfbs",
        "orin_image_apriltag/file/orin_image_apriltag.bfbs",
        "orin_large_image_apriltag/file/orin_large_gs_apriltag.bfbs",
        "apriltag_test_bfbs_images/"
        "bfbs-capture-2013-01-18_08-54-09.501047728.bfbs",
        "apriltag_test_bfbs_images/"
        "bfbs-capture-2013-01-18_08-51-24.861065764.bfbs",
        "apriltag_test_bfbs_images/"
        "bfbs-capture-2013-01-18_08-52-01.846912552.bfbs",
        "apriltag_test_bfbs_images/"
        "bfbs-capture-2013-01-18_08-52-33.462848049.bfbs",
        "apriltag_test_bfbs_images/"
        "bfbs-capture-2013-01-18_08-54-24.931661979.bfbs",
        "apriltag_test_bfbs_images/"
        "bfbs-capture-2013-01-18_09-29-16.806073486.bfbs",
        "apriltag_test_bfbs_images/"
        "bfbs-capture-2013-01-18_09-33-00.993756514.bfbs",
        "apriltag_test_bfbs_images/"
        "bfbs-capture-2013-01-18_08-57-00.171120695.bfbs",
        "apriltag_test_bfbs_images/"
        "bfbs-capture-2013-01-18_08-57-17.858752817.bfbs",
        "apriltag_test_bfbs_images/"
        "bfbs-capture-2013-01-18_08-57-08.096597542.bfbs"));

// Tests that the filter coefficients produced by FilterCoefficients() match the
// implementation in AprilRobotics.
TEST(FilterTest, Matches) {
  constexpr double sigma = 1;  // was 3

  // cutoff = exp(-j*j/(2*sigma*sigma));
  // log(cutoff) = -j*j / (2*sigma*sigma)
  // log(cutoff)*2*sigma*sigma = -j*j;

  // how big a filter should we use? We make our kernel big
  // enough such that we represent any values larger than
  // 'cutoff'.

  // XXX Tunable (though not super useful to change)
  constexpr double cutoff = 0.05;
  int fsz = sqrt(-log(cutoff) * 2 * sigma * sigma) + 1;
  fsz = 2 * fsz + 1;

  // For default values of cutoff = 0.05, sigma = 3,
  // we have fsz = 17.
  std::vector<float> f;

  for (int i = 0; i < fsz; i++) {
    const int j = i - fsz / 2;
    f.emplace_back(exp(-j * j / (2 * sigma * sigma)));
  }

  constexpr std::array<float, 7> coefficients = FilterCoefficients();
  ASSERT_EQ(coefficients.size(), f.size());

  for (size_t i = 0; i < f.size(); ++i) {
    EXPECT_EQ(coefficients[i], f[i]);
  }
}

// Tests that Unrank returns the right value for each input.
TEST(FilterTest, Unrank) {
  const int nmaxima = 10;
  int overall_count = 0;
  for (int m0 = 0; m0 < nmaxima - 3; m0++) {
    for (int m1 = m0 + 1; m1 < nmaxima - 2; m1++) {
      for (int m2 = m1 + 1; m2 < nmaxima - 1; m2++) {
        for (int m3 = m2 + 1; m3 < nmaxima; m3++) {
          const std::tuple<int, int, int, int> unranked = Unrank(overall_count);

          VLOG(1) << overall_count << " -> [" << m0 << ", " << m1 << ", " << m2
                  << ", " << m3 << "] -> [" << std::get<0>(unranked) << ", "
                  << std::get<1>(unranked) << ", " << std::get<2>(unranked)
                  << ", " << std::get<3>(unranked) << "]";
          ASSERT_EQ(m0, std::get<0>(unranked));
          ASSERT_EQ(m1, std::get<1>(unranked));
          ASSERT_EQ(m2, std::get<2>(unranked));
          ASSERT_EQ(m3, std::get<3>(unranked));
          ++overall_count;
        }
      }
    }
  }
  EXPECT_EQ(overall_count, MaxRankedIndex());
}

// Tests our Undistort is working properly
TEST_F(AprilDetectionTest, Undistort) {
  auto image = ReadImage("orin_capture_24_04/file/orin_capture_24_04.bfbs");

  LOG(INFO) << "Image is: " << image.message().cols() << " x "
            << image.message().rows();

  CudaAprilTagDetector cuda_detector(image.message().cols(),
                                     image.message().rows(), tag36h11_create());

  cuda_detector.set_undistort(true);

  const cv::Mat color_image = ToMat(&image.message());

  cuda_detector.SetCameraFourConstants();

  cuda_detector.DetectGPU(color_image.clone());
  cuda_detector.DetectCPU(color_image.clone());
  cuda_detector.Check(color_image.clone());
  if (absl::GetFlag(FLAGS_debug)) {
    cuda_detector.WriteDebug(color_image);
  }
}

// Tests our Undistort is working properly with a tag at the edge of the image
TEST_F(AprilDetectionTest, UndistortEdge) {
  auto image =
      ReadImage("orin_capture_24_04_side/file/orin_capture_24_04_side.bfbs");

  LOG(INFO) << "Image is: " << image.message().cols() << " x "
            << image.message().rows();

  CudaAprilTagDetector cuda_detector(image.message().cols(),
                                     image.message().rows(), tag36h11_create());

  cuda_detector.set_undistort(true);

  const cv::Mat color_image = ToMat(&image.message());

  cuda_detector.SetCameraFourConstants();

  cuda_detector.DetectGPU(color_image.clone());
  cuda_detector.DetectCPU(color_image.clone());
  cuda_detector.Check(color_image.clone());
  if (absl::GetFlag(FLAGS_debug)) {
    cuda_detector.WriteDebug(color_image);
  }
}
}  // namespace frc971::apriltag::testing
