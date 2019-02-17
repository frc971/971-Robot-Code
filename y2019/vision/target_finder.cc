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

// TODO: Try hierarchical merge for this.
// Convert blobs into polygons.
std::vector<aos::vision::Segment<2>> TargetFinder::FillPolygon(
    const RangeImage &blob, bool verbose) {
  if (verbose) printf("Process Polygon.\n");
  alloc_.reset();
  auto *st = RangeImgToContour(blob, &alloc_);

  struct Pt {
    float x;
    float y;
  };
  std::vector<Pt> pts;

  // Collect all slopes from the contour.
  auto opt = st->pt;
  for (auto *node = st; node->next != st;) {
    node = node->next;

    auto npt = node->pt;

    pts.push_back(
        {static_cast<float>(npt.x - opt.x), static_cast<float>(npt.y - opt.y)});

    opt = npt;
  }

  const int n = pts.size();
  auto get_pt = [&](int i) { return pts[(i + n * 2) % n]; };

  std::vector<Pt> pts_new = pts;
  auto run_box_filter = [&](int window_size) {
    for (size_t i = 0; i < pts.size(); ++i) {
      Pt a{0.0, 0.0};
      for (int j = -window_size; j <= window_size; ++j) {
        Pt p = get_pt(j + i);
        a.x += p.x;
        a.y += p.y;
      }
      a.x /= (window_size * 2 + 1);
      a.y /= (window_size * 2 + 1);

      float scale = 1.0 + (i / float(pts.size() * 10));
      a.x *= scale;
      a.y *= scale;
      pts_new[i] = a;
    }
    pts = pts_new;
  };
  // Three box filter makith a guassian?
  // Run gaussian filter over the slopes.
  run_box_filter(2);
  run_box_filter(2);
  run_box_filter(2);

  // Heuristic which says if a particular slope is part of a corner.
  auto is_corner = [&](size_t i) {
    Pt a = get_pt(i - 3);
    Pt b = get_pt(i + 3);
    double dx = (a.x - b.x);
    double dy = (a.y - b.y);
    return dx * dx + dy * dy > 0.25;
  };

  bool prev_v = is_corner(-1);

  // Find all centers of corners.
  // Because they round, multiple points may be a corner.
  std::vector<size_t> edges;
  size_t kBad = pts.size() + 10;
  size_t prev_up = kBad;
  size_t wrapped_n = prev_up;

  for (size_t i = 0; i < pts.size(); ++i) {
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
    edges.push_back(((prev_up + pts.size() + wrapped_n - 1) / 2) % pts.size());
  }

  if (verbose) printf("Edge Count (%zu).\n", edges.size());

  // Get all CountourNodes from the contour.
  using aos::vision::PixelRef;
  std::vector<ContourNode *> segments;
  {
    std::vector<ContourNode *> segments_all;

    for (ContourNode *node = st; node->next != st;) {
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
      auto *ed = segments[(i + 1) % segments.size()];
      auto *st = segments[i];
      float mx = 0.0;
      float my = 0.0;
      int n = 0;
      for (auto *node = st; node != ed; node = node->next) {
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
      for (auto *node = st; node != ed; node = node->next) {
        float x = node->pt.x - mx;
        float y = node->pt.y - my;
        xx += x * x;
        xy += x * y;
        yy += y * y;
      }

      // TODO: Extract common to hierarchical merge.
      float neg_b_over_2 = (xx + yy) / 2.0;
      float c = (xx * yy - xy * xy);

      float sqr = sqrt(neg_b_over_2 * neg_b_over_2 - c);

      {
        float lam = neg_b_over_2 + sqr;
        float x = xy;
        float y = lam - xx;

        float norm = sqrt(x * x + y * y);
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
  for (const auto &poly : seg_list) {
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
