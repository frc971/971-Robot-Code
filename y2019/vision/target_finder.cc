#include "y2019/vision/target_finder.h"

#include "aos/vision/blob/hierarchical_contour_merge.h"

using namespace aos::vision;

namespace y2019 {
namespace vision {

TargetFinder::TargetFinder() : target_template_(Target::MakeTemplate()) {}

aos::vision::RangeImage TargetFinder::Threshold(aos::vision::ImagePtr image) {
  const uint8_t threshold_value = GetThresholdValue();
  return aos::vision::DoThreshold(image, [&](aos::vision::PixelRef &px) {
    if (px.g > threshold_value && px.b > threshold_value &&
        px.r > threshold_value) {
      return true;
    }
    return false;
  });
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

ContourNode* TargetFinder::GetContour(const RangeImage &blob) {
  alloc_.reset();
  return RangeImgToContour(blob, &alloc_);
}

// TODO(ben): These values will be moved into the constants.h file.
namespace {

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
    const double coeff =
        1.0 + r_sqr * (k_1 + k_2 * r_sqr * (1.0 + k_3 * r_sqr));
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
std::vector<aos::vision::Segment<2>> TargetFinder::FillPolygon(
    const ::std::vector<::Eigen::Vector2f> &contour, bool verbose) {
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

  ::std::vector<::Eigen::Vector2f> filtered_slopes = slopes;
  // Three box filter makith a guassian?
  // Run gaussian filter over the slopes 3 times.  That'll get us pretty close
  // to running a gausian over it.
  for (int k = 0; k < 3; ++k) {
    const int window_size = 2;
    for (size_t i = 0; i < slopes.size(); ++i) {
      ::Eigen::Vector2f a = ::Eigen::Vector2f::Zero();
      for (int j = -window_size; j <= window_size; ++j) {
        ::Eigen::Vector2f p = get_pt(j + i);
        a += p;
      }
      a /= (window_size * 2 + 1);

      const float scale = 1.0 + (i / float(slopes.size() * 10));
      a *= scale;
      filtered_slopes[i] = a;
    }
    slopes = filtered_slopes;
  }

  // Heuristic which says if a particular slope is part of a corner.
  auto is_corner = [&](size_t i) {
    const ::Eigen::Vector2f a = get_pt(i - 3);
    const ::Eigen::Vector2f b = get_pt(i + 3);
    const double dx = (a.x() - b.x());
    const double dy = (a.y() - b.y());
    return dx * dx + dy * dy > 0.25;
  };

  bool prev_v = is_corner(-1);

  // Find all centers of corners.
  // Because they round, multiple slopes may be a corner.
  ::std::vector<size_t> edges;
  const size_t kBad = slopes.size() + 10;
  size_t prev_up = kBad;
  size_t wrapped_n = prev_up;

  for (size_t i = 0; i < slopes.size(); ++i) {
    bool v = is_corner(i);
    if (prev_v && !v) {
      if (prev_up == kBad) {
        wrapped_n = i;
      } else {
        edges.push_back((prev_up + i - 1) / 2);
      }
    }
    if (v && !prev_v) {
      prev_up = i;
    }
    prev_v = v;
  }

  if (wrapped_n != kBad) {
    edges.push_back(((prev_up + slopes.size() + wrapped_n - 1) / 2) % slopes.size());
  }

  if (verbose) printf("Edge Count (%zu).\n", edges.size());

  // Run best-fits over each line segment.
  ::std::vector<Segment<2>> seg_list;
  if (edges.size() == 4) {
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

        seg_list.push_back(
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
  if (verbose) printf("Poly Count (%zu).\n", seg_list.size());
  return seg_list;
}

// Convert segments into target components (left or right)
std::vector<TargetComponent> TargetFinder::FillTargetComponentList(
    const std::vector<std::vector<Segment<2>>> &seg_list) {
  std::vector<TargetComponent> list;
  TargetComponent new_target;
  for (const std::vector<Segment<2>> &poly : seg_list) {
    // Reject missized pollygons for now. Maybe rectify them here in the future;
    if (poly.size() != 4) {
      continue;
    }
    std::vector<Vector<2>> corners;
    for (size_t i = 0; i < 4; ++i) {
      Vector<2> corner = poly[i].Intersect(poly[(i + 1) % 4]);
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
    std::pair<size_t, size_t> closest;
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

    // This piece of the target should be ready now.
    list.emplace_back(new_target);
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

      // Reject targets that are too far off vertically.
      Vector<2> a_center = a.major_axis.Center();
      if (a_center.y() > b.bottom.y() || a_center.y() < b.top.y()) {
        continue;
      }
      Vector<2> b_center = b.major_axis.Center();
      if (b_center.y() > a.bottom.y() || b_center.y() < a.top.y()) {
        continue;
      }

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
      }
      if (target_valid) {
        target_list.emplace_back(new_target);
      }
    }
  }
  if (verbose) printf("Possible Target: %zu.\n", target_list.size());
  return target_list;
}

std::vector<IntermediateResult> TargetFinder::FilterResults(
    const std::vector<IntermediateResult> &results, uint64_t print_rate) {
  std::vector<IntermediateResult> filtered;
  for (const IntermediateResult &res : results) {
    if (res.solver_error < 75.0) {
      filtered.emplace_back(res);
    }
  }
  frame_count_++;
  if (!filtered.empty()) {
    valid_result_count_++;
  }
  if (print_rate > 0 && frame_count_ > print_rate) {
    LOG(INFO, "Found (%zu / %zu)(%.2f) targets.\n", valid_result_count_,
        frame_count_, (double)valid_result_count_ / (double)frame_count_);
    frame_count_ = 0;
    valid_result_count_ = 0;
  }

  return filtered;
}

}  // namespace vision
}  // namespace y2019
