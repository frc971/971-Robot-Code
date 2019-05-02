#include "y2019/vision/target_finder.h"

#include "aos/vision/blob/hierarchical_contour_merge.h"
#include "ceres/ceres.h"

using namespace aos::vision;

namespace y2019 {
namespace vision {

TargetFinder::TargetFinder()
    : target_template_(Target::MakeTemplate()),
      ceres_context_(ceres::Context::Create()) {}

TargetFinder::~TargetFinder() {}

aos::vision::RangeImage TargetFinder::Threshold(aos::vision::ImagePtr image) {
  const uint8_t threshold_value = GetThresholdValue();
  return aos::vision::ThresholdImageWithFunction(
      image, [&](aos::vision::PixelRef px) {
        if (px.g > threshold_value && px.b > threshold_value &&
            px.r > threshold_value) {
          return true;
        }
        return false;
      });
}

int TargetFinder::PixelCount(BlobList *imgs) {
  int num_pixels = 0;
  for (RangeImage &img : *imgs) {
    num_pixels += img.npixels();
  }
  return num_pixels;
}

// Filter blobs on size.
void TargetFinder::PreFilter(BlobList *imgs) {
  imgs->erase(
      std::remove_if(imgs->begin(), imgs->end(),
                     [](RangeImage &img) {
                       // We can drop images with a small number of
                       // pixels, but images
                       // must be over 20px or the math will have issues.
                       return (img.npixels() < 100 || img.height() < 25);
                     }),
      imgs->end());
}

ContourNode *TargetFinder::GetContour(const RangeImage &blob) {
  alloc_.reset();
  return RangeImgToContour(blob, &alloc_);
}

// TODO(ben): These values will be moved into the constants.h file.
namespace {

::Eigen::Vector2f AosVectorToEigenVector(Vector<2> in) {
  return ::Eigen::Vector2f(in.x(), in.y());
}

constexpr double f_x = 481.4957;
constexpr double c_x = 341.215;
constexpr double f_y = 484.314;
constexpr double c_y = 251.29;

constexpr double f_x_prime = 363.1424;
constexpr double c_x_prime = 337.9895;
constexpr double f_y_prime = 366.4837;
constexpr double c_y_prime = 240.0702;

constexpr double k_1 = -0.2739;
constexpr double k_2 = 0.01583;
constexpr double k_3 = 0.04201;

constexpr int iterations = 7;

}

::Eigen::Vector2f UnWarpPoint(const Point point) {
  const double x0 = ((double)point.x - c_x) / f_x;
  const double y0 = ((double)point.y - c_y) / f_y;
  double x = x0;
  double y = y0;
  for (int i = 0; i < iterations; i++) {
    const double r_sqr = x * x + y * y;
    const double coeff = 1.0 + r_sqr * (k_1 + r_sqr * (k_2 + r_sqr * (k_3)));
    x = x0 / coeff;
    y = y0 / coeff;
  }
  const double nx = x * f_x_prime + c_x_prime;
  const double ny = y * f_y_prime + c_y_prime;
  return ::Eigen::Vector2f(nx, ny);
}

::std::vector<::Eigen::Vector2f> TargetFinder::UnWarpContour(
    ContourNode *start) const {
  ::std::vector<::Eigen::Vector2f> result;
  ContourNode *node = start;
  while (node->next != start) {
    result.push_back(UnWarpPoint(node->pt));
    node = node->next;
  }
  result.push_back(UnWarpPoint(node->pt));
  return result;
}

// TODO: Try hierarchical merge for this.
// Convert blobs into polygons.
Polygon TargetFinder::FindPolygon(::std::vector<::Eigen::Vector2f> &&contour,
                                  bool verbose) {
  if (verbose) printf("Process Polygon.\n");

  ::std::vector<::Eigen::Vector2f> slopes;

  // Collect all slopes from the contour.
  ::Eigen::Vector2f previous_point = contour[0];
  for (size_t i = 0; i < contour.size(); ++i) {
    ::Eigen::Vector2f next_point = contour[(i + 1) % contour.size()];

    slopes.push_back(next_point - previous_point);

    previous_point = next_point;
  }

  const int num_points = slopes.size();
  auto get_pt = [&slopes, num_points](int i) {
    return slopes[(i + num_points * 2) % num_points];
  };

  // Bigger objects should be more filtered.  Filter roughly proportional to the
  // perimeter of the object.
  const int range = slopes.size() / 50;
  if (verbose) printf("Corner range: %d.\n", range);

  ::std::vector<::Eigen::Vector2f> filtered_slopes = slopes;
  // Three box filter makith a guassian?
  // Run gaussian filter over the slopes 3 times.  That'll get us pretty close
  // to running a gausian over it.
  for (int k = 0; k < 3; ++k) {
    const int window_size = ::std::max(2, range);
    for (size_t i = 0; i < slopes.size(); ++i) {
      ::Eigen::Vector2f a = ::Eigen::Vector2f::Zero();
      for (int j = -window_size; j <= window_size; ++j) {
        ::Eigen::Vector2f p = get_pt(j + i);
        a += p;
      }
      a /= (window_size * 2 + 1);

      filtered_slopes[i] = a;
    }
    slopes = filtered_slopes;
  }
  if (verbose) printf("Point count: %zu.\n", slopes.size());

  ::std::vector<float> corner_metric(slopes.size(), 0.0);

  for (size_t i = 0; i < slopes.size(); ++i) {
    const ::Eigen::Vector2f a = get_pt(i - ::std::max(3, range));
    const ::Eigen::Vector2f b = get_pt(i + ::std::max(3, range));
    corner_metric[i] = (a - b).squaredNorm();
  }

  // We want to find the Nth highest peaks.
  // Clever algorithm: Find the highest point.  Then, walk forwards and
  // backwards to find the next valley each direction which is over x% lower
  // than the peak.
  // We want to ignore those points in the future.  Set them to 0.
  // Repeat until we've found the Nth highest peak.

  // Find all centers of corners.
  // Because they round, multiple slopes may be a corner.
  ::std::vector<size_t> edges;

  constexpr float peak_acceptance_ratio = 0.16;
  constexpr float valley_ratio = 0.75;

  float highest_peak_value = 0.0;

  // Nth higest points.
  while (edges.size() < 5) {
    const ::std::vector<float>::iterator max_element =
        ::std::max_element(corner_metric.begin(), corner_metric.end());
    const size_t highest_index =
        ::std::distance(corner_metric.begin(), max_element);
    const float max_value = *max_element;
    if (edges.size() == 0) {
      highest_peak_value = max_value;
    }
    if (max_value < highest_peak_value * peak_acceptance_ratio &&
        edges.size() == 4) {
      if (verbose)
        printf("Rejecting index: %zu, %f (%f %%)\n", highest_index, max_value,
               max_value / highest_peak_value);
      break;
    }
    const float valley_value = max_value * valley_ratio;

    if (verbose)
      printf("Highest index: %zu, %f (%f %%)\n", highest_index, max_value,
             max_value / highest_peak_value);

    bool foothill = false;
    {
      float min_value = max_value;
      size_t fwd_index = (highest_index + 1) % corner_metric.size();
      while (true) {
        const float current_value = corner_metric[fwd_index];

        if (current_value == -1.0) {
          if (min_value >= valley_value) {
            if (verbose) printf("Foothill\n");
            foothill = true;
          }
          break;
        }

        min_value = ::std::min(current_value, min_value);

        if (min_value < valley_value && current_value > min_value) {
          break;
        }
        // Kill!!!
        corner_metric[fwd_index] = -1.0;

        fwd_index = (fwd_index + 1) % corner_metric.size();
      }
    }

    {
      float min_value = max_value;
      size_t rev_index =
          (highest_index - 1 + corner_metric.size()) % corner_metric.size();
      while (true) {
        const float current_value = corner_metric[rev_index];

        if (current_value == -1.0) {
          if (min_value >= valley_value) {
            if (verbose) printf("Foothill\n");
            foothill = true;
          }
          break;
        }

        min_value = ::std::min(current_value, min_value);

        if (min_value < valley_value && current_value > min_value) {
          break;
        }
        // Kill!!!
        corner_metric[rev_index] = -1.0;

        rev_index =
            (rev_index - 1 + corner_metric.size()) % corner_metric.size();
      }
    }

    *max_element = -1.0;
    if (!foothill) {
      edges.push_back(highest_index);
    }
  }

  ::std::sort(edges.begin(), edges.end());

  if (verbose) printf("Edge Count (%zu).\n", edges.size());

  // Run best-fits over each line segment.
  Polygon polygon;
  if (edges.size() >= 3) {
    for (size_t i = 0; i < edges.size(); ++i) {
      // Include the corners in both line fits.
      const size_t segment_start_index = edges[i];
      const size_t segment_end_index =
          (edges[(i + 1) % edges.size()] + 1) % contour.size();
      float mx = 0.0;
      float my = 0.0;
      int n = 0;
      for (size_t j = segment_start_index; j != segment_end_index;
           (j = (j + 1) % contour.size())) {
        mx += contour[j].x();
        my += contour[j].y();
        ++n;
        // (x - [x] / N) ** 2 = [x * x] - 2 * [x] * [x] / N + [x] * [x] / N / N;
      }
      mx /= n;
      my /= n;

      float xx = 0.0;
      float xy = 0.0;
      float yy = 0.0;
      for (size_t j = segment_start_index; j != segment_end_index;
           (j = (j + 1) % contour.size())) {
        const float x = contour[j].x() - mx;
        const float y = contour[j].y() - my;
        xx += x * x;
        xy += x * y;
        yy += y * y;
      }

      // TODO: Extract common to hierarchical merge.
      const float neg_b_over_2 = (xx + yy) / 2.0;
      const float c = (xx * yy - xy * xy);

      const float sqr = sqrt(neg_b_over_2 * neg_b_over_2 - c);

      {
        const float lam = neg_b_over_2 + sqr;
        float x = xy;
        float y = lam - xx;

        const float norm = hypot(x, y);
        x /= norm;
        y /= norm;

        polygon.segments.push_back(
            Segment<2>(Vector<2>(mx, my), Vector<2>(mx + x, my + y)));
      }

      /* Characteristic polynomial
      1 lam^2 - (xx + yy) lam + (xx * yy - xy * xy) = 0

      [a b]
      [c d]

      // covariance matrix.
      [xx xy] [nx]
      [xy yy] [ny]
      */
    }
  }
  if (verbose) printf("Poly Count (%zu).\n", polygon.segments.size());
  polygon.contour = ::std::move(contour);
  return polygon;
}

// Convert segments into target components (left or right)
::std::vector<TargetComponent> TargetFinder::FillTargetComponentList(
    const ::std::vector<Polygon> &seg_list, bool verbose) {
  ::std::vector<TargetComponent> list;
  TargetComponent new_target;
  for (const Polygon &polygon : seg_list) {
    // Reject missized pollygons for now. Maybe rectify them here in the future;
    if (polygon.segments.size() != 4) {
      continue;
    }
    ::std::vector<Vector<2>> corners;
    for (size_t i = 0; i < 4; ++i) {
      Vector<2> corner =
          polygon.segments[i].Intersect(polygon.segments[(i + 1) % 4]);
      if (::std::isnan(corner.x()) || ::std::isnan(corner.y())) {
        break;
      }
      corners.push_back(corner);
    }
    if (corners.size() != 4) {
      continue;
    }

    // Select the closest two points. Short side of the rectangle.
    double min_dist = -1;
    ::std::pair<size_t, size_t> closest;
    for (size_t i = 0; i < 4; ++i) {
      size_t next = (i + 1) % 4;
      double nd = corners[i].SquaredDistanceTo(corners[next]);
      if (min_dist == -1 || nd < min_dist) {
        min_dist = nd;
        closest.first = i;
        closest.second = next;
      }
    }

    // Verify our top is above the bottom.
    size_t bot_index = closest.first;
    size_t top_index = (closest.first + 2) % 4;
    if (corners[top_index].y() < corners[bot_index].y()) {
      closest.first = top_index;
      closest.second = (top_index + 1) % 4;
    }

    // Find the major axis.
    size_t far_first = (closest.first + 2) % 4;
    size_t far_second = (closest.second + 2) % 4;
    Segment<2> major_axis(
        (corners[closest.first] + corners[closest.second]) * 0.5,
        (corners[far_first] + corners[far_second]) * 0.5);
    if (major_axis.AsVector().AngleToZero() > M_PI / 180.0 * 120.0 ||
        major_axis.AsVector().AngleToZero() < M_PI / 180.0 * 60.0) {
      // Target is angled way too much, drop it.
      continue;
    }

    // organize the top points.
    Vector<2> topA = corners[closest.first] - major_axis.B();
    new_target.major_axis = major_axis;
    if (major_axis.AsVector().AngleToZero() > M_PI / 2.0) {
      // We have a left target since we are leaning positive.
      new_target.is_right = false;
      if (topA.AngleTo(major_axis.AsVector()) > 0.0) {
        // And our A point is left of the major axis.
        new_target.inside = corners[closest.second];
        new_target.top = corners[closest.first];
      } else {
        // our A point is to the right of the major axis.
        new_target.inside = corners[closest.first];
        new_target.top = corners[closest.second];
      }
    } else {
      // We have a right target since we are leaning negative.
      new_target.is_right = true;
      if (topA.AngleTo(major_axis.AsVector()) > 0.0) {
        // And our A point is left of the major axis.
        new_target.inside = corners[closest.first];
        new_target.top = corners[closest.second];
      } else {
        // our A point is to the right of the major axis.
        new_target.inside = corners[closest.second];
        new_target.top = corners[closest.first];
      }
    }

    // organize the top points.
    Vector<2> botA = corners[far_first] - major_axis.A();
    if (major_axis.AsVector().AngleToZero() > M_PI / 2.0) {
      // We have a right target since we are leaning positive.
      if (botA.AngleTo(major_axis.AsVector()) < M_PI) {
        // And our A point is left of the major axis.
        new_target.outside = corners[far_second];
        new_target.bottom = corners[far_first];
      } else {
        // our A point is to the right of the major axis.
        new_target.outside = corners[far_first];
        new_target.bottom = corners[far_second];
      }
    } else {
      // We have a left target since we are leaning negative.
      if (botA.AngleTo(major_axis.AsVector()) < M_PI) {
        // And our A point is left of the major axis.
        new_target.outside = corners[far_first];
        new_target.bottom = corners[far_second];
      } else {
        // our A point is to the right of the major axis.
        new_target.outside = corners[far_second];
        new_target.bottom = corners[far_first];
      }
    }

    // Take the vector which points from the bottom to the top of the target
    // along the outside edge.
    const ::Eigen::Vector2f outer_edge_vector =
        AosVectorToEigenVector(new_target.top - new_target.outside);
    // Now, dot each point in the perimeter along this vector.  The one with the
    // smallest component will be the one closest to the bottom along this
    // direction vector.
    ::Eigen::Vector2f smallest_point = polygon.contour[0];
    float smallest_value = outer_edge_vector.transpose() * smallest_point;
    for (const ::Eigen::Vector2f point : polygon.contour) {
      const float current_value = outer_edge_vector.transpose() * point;
      if (current_value < smallest_value) {
        smallest_value = current_value;
        smallest_point = point;
      }
    }

    // This piece of the target should be ready now.
    new_target.bottom_point = smallest_point;
    if (verbose) {
      printf("Lowest point in the blob is (%f, %f)\n", smallest_point.x(),
             smallest_point.y());
    }

    // This piece of the target should be ready now.
    list.emplace_back(new_target);

    if (verbose) printf("Happy with a target\n");
  }

  return list;
}

// Match components into targets.
std::vector<Target> TargetFinder::FindTargetsFromComponents(
    const std::vector<TargetComponent> component_list, bool verbose) {
  std::vector<Target> target_list;
  using namespace aos::vision;
  if (component_list.size() < 2) {
    // We don't enough parts for a target.
    return target_list;
  }

  for (size_t i = 0; i < component_list.size(); i++) {
    const TargetComponent &a = component_list[i];
    for (size_t j = 0; j < i; j++) {
      bool target_valid = false;
      Target new_target;
      const TargetComponent &b = component_list[j];

      if (a.is_right && !b.is_right) {
        if (a.top.x() > b.top.x()) {
          new_target.right = a;
          new_target.left = b;
          target_valid = true;
        }
      } else if (!a.is_right && b.is_right) {
        if (b.top.x() > a.top.x()) {
          new_target.right = b;
          new_target.left = a;
          target_valid = true;
        }
      } else if (verbose) {
        printf("Found same side components: %s.\n",
               a.is_right ? "right" : "left");
      }
      if (target_valid) {
        target_list.emplace_back(new_target);
      }
    }
  }
  if (verbose) printf("Possible Target: %zu.\n", target_list.size());
  return target_list;
}

bool TargetFinder::MaybePickAndUpdateResult(IntermediateResult *result,
                                            bool verbose) {
  // Based on a linear regression between error and distance to target.
  // Closer targets can have a higher error because they are bigger.
  const double acceptable_error =
      std::max(2 * (75 - 12 * result->extrinsics.z), 75.0);
  if (!result->good_corners) {
    if (verbose) {
      printf("Rejecting a target with bad corners: (%f, %f)\n",
             result->solver_error, result->backup_solver_error);
    }
  } else if (result->solver_error < acceptable_error) {
    if (verbose) {
      printf("Using an 8 point solve: %f < %f \n", result->solver_error,
             acceptable_error);
    }
    return true;
  } else if (result->backup_solver_error < acceptable_error) {
    if (verbose) {
      printf("Using a 4 point solve: %f < %f \n", result->backup_solver_error,
             acceptable_error);
    }
    IntermediateResult backup;
    result->extrinsics = result->backup_extrinsics;
    result->solver_error = result->backup_solver_error;
    return true;
  } else if (verbose) {
    printf("Rejecting a target with errors: (%f, %f) > %f \n",
           result->solver_error, result->backup_solver_error, acceptable_error);
  }
  return false;
}

std::vector<IntermediateResult> TargetFinder::FilterResults(
    const std::vector<IntermediateResult> &results, uint64_t print_rate,
    bool verbose) {
  std::vector<IntermediateResult> filtered;
  for (const IntermediateResult &res : results) {
    IntermediateResult updatable_result = res;
    if (MaybePickAndUpdateResult(&updatable_result, verbose)) {
      filtered.emplace_back(updatable_result);
    }
  }

  // Sort the target list so that the widest (ie closest) target is first.
  sort(filtered.begin(), filtered.end(),
       [](const IntermediateResult &a, const IntermediateResult &b)
           -> bool { return a.target_width > b.target_width; });

  frame_count_++;
  if (!filtered.empty()) {
    valid_result_count_++;
  }
  if (print_rate > 0 && frame_count_ > print_rate) {
    LOG(INFO) << "Found (" << valid_result_count_ << " / " << frame_count_
              << ")(" << ((double)valid_result_count_ / (double)frame_count_)
              << " targets.";
    frame_count_ = 0;
    valid_result_count_ = 0;
  }

  return filtered;
}

bool TargetFinder::TestExposure(const std::vector<IntermediateResult> &results,
                                int pixel_count, int *desired_exposure) {
  // TODO(ben): Add these values to config file.
  constexpr double low_dist = 0.8;
  constexpr int low_exposure  = 60;
  constexpr int mid_exposure  = 200;

  bool needs_update = false;
  if (results.size() > 0) {
    // We are seeing a target so lets use an exposure
    // based on the distance to that target.
    // First result should always be the closest target.
    if (results[0].extrinsics.z < low_dist) {
      LOG(INFO) << "Low exposure";
      *desired_exposure = low_exposure;
      close_bucket_ = 4;
    } else {
      LOG(INFO) << "Mid exposure";
      *desired_exposure = mid_exposure;
    }
    if (*desired_exposure != current_exposure_) {
      needs_update = true;
      current_exposure_ = *desired_exposure;
    }
  } else {
    close_bucket_ = ::std::max(0, close_bucket_ - 1);
    // It's been a while since we saw a target.
    if (close_bucket_ == 0) {
      if (pixel_count > 6000) {
        if (low_exposure != current_exposure_) {
          needs_update = true;
          current_exposure_ = low_exposure;
          *desired_exposure = low_exposure;
        }
      } else {
        if (mid_exposure != current_exposure_) {
          needs_update = true;
          current_exposure_ = mid_exposure;
          *desired_exposure = mid_exposure;
        }
      }
    }
  }
  return needs_update;
}

}  // namespace vision
}  // namespace y2019
