#include "y2019/vision/target_finder.h"

#include "aos/vision/blob/hierarchical_contour_merge.h"

using namespace aos::vision;

namespace y2019 {
namespace vision {

TargetFinder::TargetFinder() { target_template_ = Target::MakeTemplate(); }

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

// TODO(ben): These values will be moved into a configuration file.
namespace {

constexpr double mtx00 = 481.4957;
constexpr double mtx02 = 341.215;
constexpr double mtx11 = 484.314;
constexpr double mtx12 = 251.29;

constexpr double new_cam00 = 363.1424;
constexpr double new_cam02 = 337.9895;
constexpr double new_cam11 = 366.4837;
constexpr double new_cam12 = 240.0702;

constexpr double dist00 = -0.2739;
constexpr double dist01 = 0.01583;
constexpr double dist04 = 0.04201;

constexpr int iterations = 7;

}

Point UnWarpPoint(const Point &point, int iterations) {
  const double x0 = ((double)point.x - mtx02) / mtx00;
  const double y0 = ((double)point.y - mtx12) / mtx11;
  double x = x0;
  double y = y0;
  for (int i = 0; i < iterations; i++) {
    const double r_sqr = x * x + y * y;
    const double coeff =
        1.0 + r_sqr * (dist00 + dist01 * r_sqr * (1.0 + dist04 * r_sqr));
    x = x0 / coeff;
    y = y0 / coeff;
  }
  double nx = x * new_cam00 + new_cam02;
  double ny = y * new_cam11 + new_cam12;
  Point p = {static_cast<int>(nx), static_cast<int>(ny)};
  return p;
}

void TargetFinder::UnWarpContour(ContourNode *start) const {
  ContourNode *node = start;
  while (node->next != start) {
    node->set_point(UnWarpPoint(node->pt, iterations));
    node = node->next;
  }
  node->set_point(UnWarpPoint(node->pt, iterations));
}

// TODO: Try hierarchical merge for this.
// Convert blobs into polygons.
std::vector<aos::vision::Segment<2>> TargetFinder::FillPolygon(
    ContourNode* start, bool verbose) {
  if (verbose) printf("Process Polygon.\n");

  struct Pt {
    float x;
    float y;
  };
  std::vector<Pt> points;

  // Collect all slopes from the contour.
  Point previous_point = start->pt;
  for (ContourNode *node = start; node->next != start;) {
    node = node->next;

    Point current_point = node->pt;

    points.push_back({static_cast<float>(current_point.x - previous_point.x),
                      static_cast<float>(current_point.y - previous_point.y)});

    previous_point = current_point;
  }

  const int num_points = points.size();
  auto get_pt = [&points, num_points](int i) {
    return points[(i + num_points * 2) % num_points];
  };

  std::vector<Pt> filtered_points = points;
  // Three box filter makith a guassian?
  // Run gaussian filter over the slopes 3 times.  That'll get us pretty close
  // to running a gausian over it.
  for (int k = 0; k < 3; ++k) {
    const int window_size = 2;
    for (size_t i = 0; i < points.size(); ++i) {
      Pt a{0.0, 0.0};
      for (int j = -window_size; j <= window_size; ++j) {
        Pt p = get_pt(j + i);
        a.x += p.x;
        a.y += p.y;
      }
      a.x /= (window_size * 2 + 1);
      a.y /= (window_size * 2 + 1);

      const float scale = 1.0 + (i / float(points.size() * 10));
      a.x *= scale;
      a.y *= scale;
      filtered_points[i] = a;
    }
    points = filtered_points;
  }

  // Heuristic which says if a particular slope is part of a corner.
  auto is_corner = [&](size_t i) {
    const Pt a = get_pt(i - 3);
    const Pt b = get_pt(i + 3);
    const double dx = (a.x - b.x);
    const double dy = (a.y - b.y);
    return dx * dx + dy * dy > 0.25;
  };

  bool prev_v = is_corner(-1);

  // Find all centers of corners.
  // Because they round, multiple points may be a corner.
  std::vector<size_t> edges;
  size_t kBad = points.size() + 10;
  size_t prev_up = kBad;
  size_t wrapped_n = prev_up;

  for (size_t i = 0; i < points.size(); ++i) {
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
    edges.push_back(((prev_up + points.size() + wrapped_n - 1) / 2) % points.size());
  }

  if (verbose) printf("Edge Count (%zu).\n", edges.size());

  // Get all CountourNodes from the contour.
  using aos::vision::PixelRef;
  std::vector<ContourNode *> segments;
  {
    std::vector<ContourNode *> segments_all;

    for (ContourNode *node = start; node->next != start;) {
      node = node->next;
      segments_all.push_back(node);
    }
    for (size_t i : edges) {
      segments.push_back(segments_all[i]);
    }
  }
  if (verbose) printf("Segment Count (%zu).\n", segments.size());

  // Run best-fits over each line segment.
  std::vector<Segment<2>> seg_list;
  if (segments.size() == 4) {
    for (size_t i = 0; i < segments.size(); ++i) {
      ContourNode *segment_end = segments[(i + 1) % segments.size()];
      ContourNode *segment_start = segments[i];
      float mx = 0.0;
      float my = 0.0;
      int n = 0;
      for (ContourNode *node = segment_start; node != segment_end;
           node = node->next) {
        mx += node->pt.x;
        my += node->pt.y;
        ++n;
        // (x - [x] / N) ** 2 = [x * x] - 2 * [x] * [x] / N + [x] * [x] / N / N;
      }
      mx /= n;
      my /= n;

      float xx = 0.0;
      float xy = 0.0;
      float yy = 0.0;
      for (ContourNode *node = segment_start; node != segment_end;
           node = node->next) {
        const float x = node->pt.x - mx;
        const float y = node->pt.y - my;
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
    if (poly.size() != 4) continue;
    std::vector<Vector<2>> corners;
    for (size_t i = 0; i < 4; ++i) {
      corners.push_back(poly[i].Intersect(poly[(i + 1) % 4]));
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
    const std::vector<IntermediateResult> &results) {
  std::vector<IntermediateResult> filtered;
  for (const IntermediateResult &res : results) {
    if (res.solver_error < 75.0) {
      filtered.emplace_back(res);
    }
  }
  return filtered;
}

}  // namespace vision
}  // namespace y2019
