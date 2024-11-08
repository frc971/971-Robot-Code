#include <cmath>
#include <vector>

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

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/time/time.h"
#include "frc971/orin/apriltag.h"
#include "frc971/orin/apriltag_input_format.h"
#include "frc971/orin/labeling_allegretti_2019_BKE.h"
#include "frc971/orin/threshold.h"

ABSL_FLAG(int32_t, debug_blob_index, 4096, "Blob to print out for");

constexpr int kUndistortIterationThreshold = 100;
constexpr double kUndistortConvergenceEpsilon = 1e-6;

extern "C" {

void quad_decode_index(apriltag_detector_t *td, struct quad *quad_original,
                       image_u8_t *im, image_u8_t *im_samples,
                       zarray_t *detections);

void reconcile_detections(zarray_t *detections, zarray_t *poly0,
                          zarray_t *poly1);
};

namespace frc971::apriltag {
namespace {

void HostFitLine(LineFitMoments moments, double *lineparam01,
                 double *lineparam23, double *err, double *mse) {
  int64_t Cxx = moments.Mxx * moments.W - static_cast<int64_t>(moments.Mx) *
                                              static_cast<int64_t>(moments.Mx);
  int64_t Cxy = moments.Mxy * moments.W - static_cast<int64_t>(moments.Mx) *
                                              static_cast<int64_t>(moments.My);
  int64_t Cyy = moments.Myy * moments.W - static_cast<int64_t>(moments.My) *
                                              static_cast<int64_t>(moments.My);

  // Pose it as an eigenvalue problem.
  const float hypot_cached = std::hypotf((Cxx - Cyy), 2 * Cxy);
  const float eight_w_squared = static_cast<float>(
      static_cast<int64_t>(moments.W) * static_cast<int64_t>(moments.W) * 8.0);
  const float eig_small = (Cxx + Cyy - hypot_cached) / eight_w_squared;

  if (lineparam01) {
    lineparam01[0] =
        static_cast<float>(moments.Mx) / static_cast<float>(moments.W * 2);
    lineparam01[1] =
        static_cast<float>(moments.My) / static_cast<float>(moments.W * 2);
  }
  if (lineparam23) {
    // These don't match the originals at all, but the math should come out
    // right.  n{xy}{12} end up being multiplied by 8 W^2, and we compare the
    // square but use hypot on nx, ny directly.  (and let the W^2 term come out
    // as common to both the hypot and nxy terms.)
    const float nx1 = static_cast<float>(Cxx - Cyy) - hypot_cached;
    const float ny1 = Cxy * 2;
    const float M1 = nx1 * nx1 + ny1 * ny1;
    const float nx2 = Cxy * 2;
    const float ny2 = static_cast<float>(Cyy - Cxx) - hypot_cached;
    const float M2 = nx2 * nx2 + ny2 * ny2;

    float nx, ny;
    if (M1 > M2) {
      nx = nx1;
      ny = ny1;
    } else {
      nx = nx2;
      ny = ny2;
    }

    float length = std::hypotf(nx, ny);
    lineparam23[0] = nx / length;
    lineparam23[1] = ny / length;
  }

  // sum of squared errors
  *err = moments.N * eig_small;

  // mean squared error
  *mse = eig_small;
}

}  // namespace

const std::vector<QuadCorners> &GpuDetector::FitQuads() const {
  return quad_corners_host_;
}

void GpuDetector::UpdateFitQuads() {
  quad_corners_host_.resize(0);
  VLOG(1) << "Considering " << fit_quads_host_.size();
  for (const FitQuad &quad : fit_quads_host_) {
    bool print = quad.blob_index == absl::GetFlag(FLAGS_debug_blob_index);
    if (!quad.valid) {
      continue;
    }
    QuadCorners corners;
    corners.blob_index = quad.blob_index;
    CHECK(normal_border_ != reversed_border_);
    corners.reversed_border = reversed_border_;

    double lines[4][4];
    for (int i = 0; i < 4; i++) {
      double err;
      double mse;
      HostFitLine(quad.moments[i], lines[i], lines[i] + 2, &err, &mse);
      if (print) {
        LOG(INFO) << "Blob " << corners.blob_index << " mse -> " << mse
                  << " err " << err << " index " << quad.indices[i] << ", "
                  << quad.indices[(i + 1) % 4];
        LOG(INFO) << "   " << quad.moments[i];
      }
    }

    bool bad_determinant = false;
    for (int i = 0; i < 4; i++) {
      // solve for the intersection of lines (i) and (i+1)&3.
      // p0 + lambda0*u0 = p1 + lambda1*u1, where u0 and u1
      // are the line directions.
      //
      // lambda0*u0 - lambda1*u1 = (p1 - p0)
      //
      // rearrange (solve for lambdas)
      //
      // [u0_x   -u1_x ] [lambda0] = [ p1_x - p0_x ]
      // [u0_y   -u1_y ] [lambda1]   [ p1_y - p0_y ]
      //
      // remember that lines[i][0,1] = p, lines[i][2,3] = NORMAL vector.
      // We want the unit vector, so we need the perpendiculars. Thus, below
      // we have swapped the x and y components and flipped the y components.

      double A00 = lines[i][3], A01 = -lines[(i + 1) & 3][3];
      double A10 = -lines[i][2], A11 = lines[(i + 1) & 3][2];
      double B0 = -lines[i][0] + lines[(i + 1) & 3][0];
      double B1 = -lines[i][1] + lines[(i + 1) & 3][1];

      double det = A00 * A11 - A10 * A01;

      // inverse.
      double W00 = A11 / det, W01 = -A01 / det;
      if (fabs(det) < 0.001) {
        bad_determinant = true;
        break;
      }

      // solve
      double L0 = W00 * B0 + W01 * B1;

      // compute intersection
      corners.corners[i][0] = lines[i][0] + L0 * A00;
      corners.corners[i][1] = lines[i][1] + L0 * A10;
      if (print) {
        LOG(INFO) << "Calculated corner[" << i << "] -> ("
                  << std::setprecision(20) << corners.corners[i][0] << ", "
                  << std::setprecision(20) << corners.corners[i][1] << ")";
      }
    }
    if (bad_determinant) {
      continue;
    }

    {
      float area = 0;

      // get area of triangle formed by points 0, 1, 2, 0
      float length[3], p;
      for (int i = 0; i < 3; i++) {
        int idxa = i;            // 0, 1, 2,
        int idxb = (i + 1) % 3;  // 1, 2, 0
        length[i] =
            hypotf((corners.corners[idxb][0] - corners.corners[idxa][0]),
                   (corners.corners[idxb][1] - corners.corners[idxa][1]));
      }
      p = (length[0] + length[1] + length[2]) / 2;

      area += sqrtf(p * (p - length[0]) * (p - length[1]) * (p - length[2]));

      // get area of triangle formed by points 2, 3, 0, 2
      for (int i = 0; i < 3; i++) {
        int idxs[] = {2, 3, 0, 2};
        int idxa = idxs[i];
        int idxb = idxs[i + 1];
        length[i] =
            hypotf((corners.corners[idxb][0] - corners.corners[idxa][0]),
                   (corners.corners[idxb][1] - corners.corners[idxa][1]));
      }
      p = (length[0] + length[1] + length[2]) / 2;

      area += sqrtf(p * (p - length[0]) * (p - length[1]) * (p - length[2]));

      if (area < 0.95 * min_tag_width_ * min_tag_width_) {
        if (print) {
          LOG(INFO) << "Area of " << area << " smaller than "
                    << 0.95 * min_tag_width_ * min_tag_width_;
        }
        continue;
      }
    }

    // reject quads whose cumulative angle change isn't equal to 2PI
    {
      bool reject_corner = false;
      for (int i = 0; i < 4; i++) {
        int i0 = i, i1 = (i + 1) & 3, i2 = (i + 2) & 3;

        float dx1 = corners.corners[i1][0] - corners.corners[i0][0];
        float dy1 = corners.corners[i1][1] - corners.corners[i0][1];
        float dx2 = corners.corners[i2][0] - corners.corners[i1][0];
        float dy2 = corners.corners[i2][1] - corners.corners[i1][1];
        float cos_dtheta =
            (dx1 * dx2 + dy1 * dy2) /
            sqrtf((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2));

        if (print) {
          LOG(INFO) << "Cosdtheta -> for " << i0 << " " << i1 << " " << i2
                    << " -> " << cos_dtheta << " threshold "
                    << tag_detector_->qtp.cos_critical_rad;
        }

        if (std::abs(cos_dtheta) > tag_detector_->qtp.cos_critical_rad ||
            dx1 * dy2 < dy1 * dx2) {
          reject_corner = true;
          break;
        }
      }
      if (reject_corner) {
        continue;
      }
    }
    quad_corners_host_.push_back(corners);
  }
}

void GpuDetector::AdjustCenter(float corners[4][2]) const {
  const float quad_decimate = tag_detector_->quad_decimate;
  if (tag_detector_->quad_decimate > 1) {
    if (tag_detector_->quad_decimate == 1.5) {
      for (int j = 0; j < 4; j++) {
        corners[j][0] *= quad_decimate;
        corners[j][1] *= quad_decimate;
      }
    } else {
      for (int j = 0; j < 4; j++) {
        corners[j][0] = (corners[j][0] - 0.5f) * quad_decimate + 0.5f;
        corners[j][1] = (corners[j][1] - 0.5f) * quad_decimate + 0.5f;
      }
    }
  }
}

void GpuDetector::AdjustPixelCenters() {
  const float quad_decimate = tag_detector_->quad_decimate;

  if (quad_decimate > 1) {
    if (tag_detector_->quad_decimate == 1.5) {
      for (QuadCorners &quad : quad_corners_host_) {
        for (int j = 0; j < 4; j++) {
          quad.corners[j][0] *= quad_decimate;
          quad.corners[j][1] *= quad_decimate;
        }
      }
    } else {
      for (QuadCorners &quad : quad_corners_host_) {
        for (int j = 0; j < 4; j++) {
          quad.corners[j][0] =
              (quad.corners[j][0] - 0.5f) * quad_decimate + 0.5f;
          quad.corners[j][1] =
              (quad.corners[j][1] - 0.5f) * quad_decimate + 0.5f;
        }
      }
    }
  }
}

static inline int detection_compare_function(const void *_a, const void *_b) {
  apriltag_detection_t *a = *(apriltag_detection_t **)_a;
  apriltag_detection_t *b = *(apriltag_detection_t **)_b;
  return a->id - b->id;
}

struct QuadDecodeTaskStruct {
  int i0, i1;
  QuadCorners *quads;
  apriltag_detector_t *td;

  image_u8_t *im;
  zarray_t *detections;

  image_u8_t *im_samples;

  CameraMatrix *camera_matrix;
  DistCoeffs *distortion_coefficients;
};

// Dewarps points from the image based on various constants
// Algorithm mainly taken from
// https://docs.opencv.org/4.0.0/d9/d0c/group__calib3d.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a
void ReDistort(double *x, double *y, CameraMatrix *camera_matrix,
               DistCoeffs *distortion_coefficients) {
  double k1 = distortion_coefficients->k1;
  double k2 = distortion_coefficients->k2;
  double p1 = distortion_coefficients->p1;
  double p2 = distortion_coefficients->p2;
  double k3 = distortion_coefficients->k3;

  double fx = camera_matrix->fx;
  double cx = camera_matrix->cx;
  double fy = camera_matrix->fy;
  double cy = camera_matrix->cy;

  double xP = (*x - cx) / fx;
  double yP = (*y - cy) / fy;

  double rSq = xP * xP + yP * yP;

  double linCoef = 1 + k1 * rSq + k2 * rSq * rSq + k3 * rSq * rSq * rSq;
  double xPP = xP * linCoef + 2 * p1 * xP * yP + p2 * (rSq + 2 * xP * xP);
  double yPP = yP * linCoef + p1 * (rSq + 2 * yP * yP) + 2 * p2 * xP * yP;

  *x = xPP * fx + cx;
  *y = yPP * fy + cy;
}

// We're undistorting using math found from this github page
// https://yangyushi.github.io/code/2020/03/04/opencv-undistort.html
bool GpuDetector::UnDistort(double *u, double *v,
                            const CameraMatrix *camera_matrix,
                            const DistCoeffs *distortion_coefficients) {
  bool converged = true;
  const double k1 = distortion_coefficients->k1;
  const double k2 = distortion_coefficients->k2;
  const double p1 = distortion_coefficients->p1;
  const double p2 = distortion_coefficients->p2;
  const double k3 = distortion_coefficients->k3;

  const double fx = camera_matrix->fx;
  const double cx = camera_matrix->cx;
  const double fy = camera_matrix->fy;
  const double cy = camera_matrix->cy;

  const double xPP = (*u - cx) / fx;
  const double yPP = (*v - cy) / fy;

  double xP = xPP;
  double yP = yPP;

  double x0 = xP;
  double y0 = yP;

  double prev_x = 0, prev_y = 0;

  int iterations = 0;
  do {
    prev_x = xP;
    prev_y = yP;
    double rSq = xP * xP + yP * yP;

    double radial_distortion =
        1 + (k1 * rSq) + (k2 * rSq * rSq) + (k3 * rSq * rSq * rSq);

    double radial_distortion_inv = 1 / radial_distortion;

    double tangential_dx = 2 * p1 * xP * yP + p2 * (rSq + k3 * rSq * rSq * rSq);
    double tangential_dy = p1 * (rSq + 2 * yP * yP) + 2 * p2 * xP * yP;

    xP = (x0 - tangential_dx) * radial_distortion_inv;
    yP = (y0 - tangential_dy) * radial_distortion_inv;

    if (iterations > kUndistortIterationThreshold) {
      converged = false;
      break;
    }

    iterations++;
  } while (std::abs(xP - prev_x) > kUndistortConvergenceEpsilon ||
           std::abs(yP - prev_y) > kUndistortConvergenceEpsilon);

  if (iterations < kUndistortIterationThreshold) {
    VLOG(1) << "Took " << iterations << " iterations to reach convergence.";
  } else {
    VLOG(1) << "Took " << iterations
            << " iterations and didn't reach convergence with "
            << " (xP, yP): "
            << " (" << xP << ", " << yP << ")"
            << " vs. (prev_x, prev_y): "
            << " (" << prev_x << ", " << prev_y << ")";
  }

  *u = xP * fx + cx;
  *v = yP * fy + cy;

  return converged;
}

// Mostly stolen from aprilrobotics, but modified to implement the dewarp.
void RefineEdges(apriltag_detector_t *td, image_u8_t *im_orig,
                 struct quad *quad, CameraMatrix *camera_matrix,
                 DistCoeffs *distortion_coefficients) {
  double lines[4][4];  // for each line, [Ex Ey nx ny]

  for (int edge = 0; edge < 4; edge++) {
    int a = edge, b = (edge + 1) & 3;  // indices of the end points.

    // compute the normal to the current line estimate
    float nx = quad->p[b][1] - quad->p[a][1];
    float ny = -quad->p[b][0] + quad->p[a][0];
    float mag = sqrtf(nx * nx + ny * ny);
    nx /= mag;
    ny /= mag;

    if (quad->reversed_border) {
      nx = -nx;
      ny = -ny;
    }

    // we will now fit a NEW line by sampling points near
    // our original line that have large gradients. On really big tags,
    // we're willing to sample more to get an even better estimate.
    int nsamples = std::max<int>(16, mag / 8);  // XXX tunable

    // stats for fitting a line...
    double Mx = 0, My = 0, Mxx = 0, Mxy = 0, Myy = 0, N = 0;

    for (int s = 0; s < nsamples; s++) {
      // compute a point along the line... Note, we're avoiding
      // sampling *right* at the corners, since those points are
      // the least reliable.
      double alpha = (1.0 + s) / (nsamples + 1);
      double x0 = alpha * quad->p[a][0] + (1 - alpha) * quad->p[b][0];
      double y0 = alpha * quad->p[a][1] + (1 - alpha) * quad->p[b][1];

      // search along the normal to this line, looking at the
      // gradients along the way. We're looking for a strong
      // response.
      double Mn = 0;
      double Mcount = 0;

      // XXX tunable: how far to search?  We want to search far
      // enough that we find the best edge, but not so far that
      // we hit other edges that aren't part of the tag. We
      // shouldn't ever have to search more than quad_decimate,
      // since otherwise we would (ideally) have started our
      // search on another pixel in the first place. Likewise,
      // for very small tags, we don't want the range to be too
      // big.
      double range = td->quad_decimate + 1;

      // XXX tunable step size.
      for (double n = -range; n <= range; n += 0.25) {
        // Because of the guaranteed winding order of the
        // points in the quad, we will start inside the white
        // portion of the quad and work our way outward.
        //
        // sample to points (x1,y1) and (x2,y2) XXX tunable:
        // how far +/- to look? Small values compute the
        // gradient more precisely, but are more sensitive to
        // noise.
        double grange = 1;
        int x1 = x0 + (n + grange) * nx;
        int y1 = y0 + (n + grange) * ny;
        if (x1 < 0 || x1 >= im_orig->width || y1 < 0 || y1 >= im_orig->height)
          continue;

        int x2 = x0 + (n - grange) * nx;
        int y2 = y0 + (n - grange) * ny;
        if (x2 < 0 || x2 >= im_orig->width || y2 < 0 || y2 >= im_orig->height)
          continue;

        int g1 = im_orig->buf[y1 * im_orig->stride + x1];
        int g2 = im_orig->buf[y2 * im_orig->stride + x2];

        if (g1 < g2)  // reject points whose gradient is "backwards". They can
                      // only hurt us.
          continue;

        double weight =
            (g2 - g1) *
            (g2 - g1);  // XXX tunable. What shape for weight=f(g2-g1)?

        // compute weighted average of the gradient at this point.
        Mn += weight * n;
        Mcount += weight;
      }

      // what was the average point along the line?
      if (Mcount == 0) continue;

      double n0 = Mn / Mcount;

      // where is the point along the line?
      double bestx = x0 + n0 * nx;
      double besty = y0 + n0 * ny;

      GpuDetector::UnDistort(&bestx, &besty, camera_matrix, distortion_coefficients);

      // update our line fit statistics
      Mx += bestx;
      My += besty;
      Mxx += bestx * bestx;
      Mxy += bestx * besty;
      Myy += besty * besty;
      N++;
    }

    // fit a line
    double Ex = Mx / N, Ey = My / N;
    double Cxx = Mxx / N - Ex * Ex;
    double Cxy = Mxy / N - Ex * Ey;
    double Cyy = Myy / N - Ey * Ey;

    // TODO: Can replace this with same code as in fit_line.
    double normal_theta = .5 * atan2f(-2 * Cxy, (Cyy - Cxx));
    nx = cosf(normal_theta);
    ny = sinf(normal_theta);
    lines[edge][0] = Ex;
    lines[edge][1] = Ey;
    lines[edge][2] = nx;
    lines[edge][3] = ny;
  }

  // now refit the corners of the quad
  for (int i = 0; i < 4; i++) {
    // solve for the intersection of lines (i) and (i+1)&3.
    double A00 = lines[i][3], A01 = -lines[(i + 1) & 3][3];
    double A10 = -lines[i][2], A11 = lines[(i + 1) & 3][2];
    double B0 = -lines[i][0] + lines[(i + 1) & 3][0];
    double B1 = -lines[i][1] + lines[(i + 1) & 3][1];

    double det = A00 * A11 - A10 * A01;

    // inverse.
    if (fabs(det) > 0.001) {
      // solve
      double W00 = A11 / det, W01 = -A01 / det;

      double L0 = W00 * B0 + W01 * B1;

      // Compute intersection. Note that line i represents the line from corner
      // i to (i+1)&3, so the intersection of line i with line (i+1)&3
      // represents corner (i+1)&3.
      double px = lines[i][0] + L0 * A00;
      double py = lines[i][1] + L0 * A10;

      ReDistort(&px, &py, camera_matrix, distortion_coefficients);
      quad->p[(i + 1) & 3][0] = px;
      quad->p[(i + 1) & 3][1] = py;
    } else {
      // this is a bad sign. We'll just keep the corner we had.
      //            debug_print("bad det: %15f %15f %15f %15f %15f\n", A00, A11,
      //            A10, A01, det);
    }
  }
}

void GpuDetector::QuadDecodeTask(void *_u) {
  QuadDecodeTaskStruct *task = reinterpret_cast<QuadDecodeTaskStruct *>(_u);
  apriltag_detector_t *td = task->td;
  image_u8_t *im = task->im;

  for (int quadidx = task->i0; quadidx < task->i1; quadidx++) {
    struct quad quad_original;
    std::memcpy(quad_original.p, task->quads[quadidx].corners,
                sizeof(task->quads[quadidx].corners));

    quad_original.reversed_border = task->quads[quadidx].reversed_border;
    quad_original.H = nullptr;
    quad_original.Hinv = nullptr;

    if (td->refine_edges) {
      RefineEdges(td, im, &quad_original, task->camera_matrix,
                  task->distortion_coefficients);
    }

    if (td->debug) {
      image_u8_t *im_quads = image_u8_copy(im);
      image_u8_darken(im_quads);
      image_u8_darken(im_quads);

      srandom(0);

      const int bias = 100;
      int color = bias + (random() % (255 - bias));

      image_u8_draw_line(im_quads, quad_original.p[0][0], quad_original.p[0][1],
                         quad_original.p[1][0], quad_original.p[1][1], color,
                         1);
      image_u8_draw_line(im_quads, quad_original.p[1][0], quad_original.p[1][1],
                         quad_original.p[2][0], quad_original.p[2][1], color,
                         1);
      image_u8_draw_line(im_quads, quad_original.p[2][0], quad_original.p[2][1],
                         quad_original.p[3][0], quad_original.p[3][1], color,
                         1);
      image_u8_draw_line(im_quads, quad_original.p[3][0], quad_original.p[3][1],
                         quad_original.p[0][0], quad_original.p[0][1], color,
                         1);

      image_u8_write_pnm(
          im_quads,
          std::string("/tmp/quad" + std::to_string(quadidx) + ".pnm").c_str());
    }

    quad_decode_index(td, &quad_original, im, task->im_samples,
                      task->detections);
  }
}

void GpuDetector::DecodeTags() {
  size_t chunksize =
      1 + quad_corners_host_.size() /
              (APRILTAG_TASKS_PER_THREAD_TARGET * tag_detector_->nthreads);

  std::vector<QuadDecodeTaskStruct> tasks(
      (quad_corners_host_.size() / chunksize + 1));

  for (int i = 0; i < zarray_size(detections_); ++i) {
    apriltag_detection_t *det;
    zarray_get(detections_, i, &det);
    apriltag_detection_destroy(det);
  }

  zarray_truncate(detections_, 0);

  after_memcpy_gray_.Synchronize();
  image_u8_t im_orig{
      .width = static_cast<int32_t>(width_),
      .height = static_cast<int32_t>(original_height_),
      .stride = static_cast<int32_t>(width_),
      .buf = const_cast<uint8_t *>(gray_image_host_ptr_), 
      // TODO - find a better way to do this --^. Problem is that 
      //        for the Mono8 case, we just want an alias to the input
      //        data, since that's already greyscale to begin with.
      //        For other input types, it should point to a class member
      //        buffer which is the output of the greyscale conversion / 
      //        d2h memcpy.
  };

  int ntasks = 0;
  for (size_t i = 0; i < quad_corners_host_.size(); i += chunksize) {
    tasks[ntasks].i0 = i;
    tasks[ntasks].i1 = std::min(quad_corners_host_.size(), i + chunksize);
    tasks[ntasks].quads = quad_corners_host_.data();
    tasks[ntasks].td = tag_detector_;
    tasks[ntasks].im = &im_orig;
    tasks[ntasks].detections = detections_;
    tasks[ntasks].camera_matrix = &camera_matrix_;
    tasks[ntasks].distortion_coefficients = &distortion_coefficients_;

    tasks[ntasks].im_samples = nullptr;

    workerpool_add_task(tag_detector_->wp, QuadDecodeTask, &tasks[ntasks]);
    ntasks++;
  }

  workerpool_run(tag_detector_->wp);

  reconcile_detections(detections_, poly0_, poly1_);

  zarray_sort(detections_, detection_compare_function);
}

}  // namespace frc971::apriltag
