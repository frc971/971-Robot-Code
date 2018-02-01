#ifndef Y2018_CONTORL_LOOPS_PYTHON_ARM_BOUNDS_H_
#define Y2018_CONTORL_LOOPS_PYTHON_ARM_BOUNDS_H_

#include <CGAL/Bbox_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/squared_distance_2.h>

#include <Eigen/Dense>

// Prototype level code to find the nearest point and distance to a polygon.

namespace y2018 {
namespace control_loops {

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef K::Segment_2 Segment;
typedef CGAL::Bbox_2 Bbox;
typedef CGAL::Polygon_2<K> SimplePolygon;
typedef CGAL::Polygon_with_holes_2<K> Polygon;
typedef K::Line_2 Line;
typedef K::Vector_2 Vector;


// Returns true if the point p3 is to the left of the vector from p1 to p2.
inline bool is_left(Point p1, Point p2, Point p3) {
  switch (CGAL::orientation(p1, p2, p3)) {
    case CGAL::LEFT_TURN:
    case CGAL::COLLINEAR:
      return true;
    case CGAL::RIGHT_TURN:
      return false;
  }
}

// Returns true if the segments intersect.
inline bool intersects(Segment s1, Segment s2) {
  return CGAL::do_intersect(s1, s2);
}

class BoundsCheck {
 public:
  BoundsCheck(const std::vector<Point> &points)
      : points_(points), grid_(points_, 6) {}

  double min_distance(Point point, ::Eigen::Matrix<double, 2, 1> *normal) const;

  const std::vector<Point> &points() const { return points_; }

 private:
  static Bbox ToBbox(const std::vector<Point> &points) {
    Bbox out;
    out += Segment(points.back(), points.front()).bbox();
    for (size_t i = 0; i < points.size() - 1; ++i) {
      out += Segment(points[i], points[i + 1]).bbox();
    }
    return out;
  }

  static SimplePolygon ToPolygon(Bbox bbox) {
    Point points[4]{{bbox.xmin(), bbox.ymin()},
                    {bbox.xmax(), bbox.ymin()},
                    {bbox.xmax(), bbox.ymax()},
                    {bbox.xmin(), bbox.ymax()}};
    return SimplePolygon(&points[0], &points[4]);
  }

  static double min_dist(Point pt, const std::vector<Point> &points,
                         Segment *best_segment) {
    *best_segment = Segment(points.back(), points.front());
    double min_dist_sqr = CGAL::squared_distance(pt, *best_segment);
    for (size_t i = 0; i < points.size() - 1; ++i) {
      Segment s(points[i], points[i + 1]);
      double segment_distance = CGAL::squared_distance(pt, s);
      if (segment_distance < min_dist_sqr) {
        min_dist_sqr = segment_distance;
        *best_segment = s;
      }
    }
    return sqrt(min_dist_sqr);
  }

  static std::vector<Segment> ToSegment(Bbox bbox) {
    Point points[4]{{bbox.xmin(), bbox.ymin()},
                    {bbox.xmax(), bbox.ymin()},
                    {bbox.xmax(), bbox.ymax()},
                    {bbox.xmin(), bbox.ymax()}};

    return std::vector<Segment>({{points[0], points[1]},
                                  {points[1], points[2]},
                                  {points[2], points[3]},
                                  {points[3], points[0]}});
  }

  static bool check_inside(Point pt, const std::vector<Point> &points) {
    switch (CGAL::bounded_side_2(&points[0], &points[points.size()], pt, K())) {
      case CGAL::ON_BOUNDED_SIDE:
      case CGAL::ON_BOUNDARY:
        return true;
      case CGAL::ON_UNBOUNDED_SIDE:
        return false;
    }
    return false;
  }

  const std::vector<Point> points_;

  class GridCell {
   public:
    GridCell(const std::vector<Point> &points, Bbox bbox) {
      bool has_intersect = false;

      Point center{(bbox.xmin() + bbox.xmax()) / 2,
                   (bbox.ymin() + bbox.ymax()) / 2};
      // Purposefully overestimate.
      double r = bbox.ymax() - bbox.ymin();

      Segment best_segment;
      double best = min_dist(center, points, &best_segment);
      dist_upper_bound_ = best + 2 * r;
      dist_lower_bound_ = std::max(best - 2 * r, 0.0);

      double sq_upper_bound = dist_upper_bound_ * dist_upper_bound_;

      auto try_add_segment = [&](Segment segment) {
        for (const auto &bbox_segment : ToSegment(bbox)) {
          if (CGAL::do_intersect(bbox_segment, segment)) {
            has_intersect = true;
          }
        }

        double dist_sqr = CGAL::squared_distance(center, segment);
        if (dist_sqr < sq_upper_bound) {
          segments_.push_back(segment);
        }
      };

      try_add_segment(Segment(points.back(), points.front()));
      for (size_t i = 0; i < points.size() - 1; ++i) {
        try_add_segment(Segment(points[i], points[i + 1]));
      }
      if (has_intersect) {
        is_borderline = true;
      } else {
        is_inside = check_inside(center, points);
      }
    }

    bool IsInside(Point pt) const {
      (void)pt;
      return is_inside;
    }

    bool IsBorderline() const { return is_borderline; }

    double DistanceSqr(Point pt, Segment *best_segment) const {
      double min_dist_sqr = CGAL::squared_distance(pt, segments_[0]);
      *best_segment = segments_[0];
      for (size_t i = 1; i < segments_.size(); ++i) {
        double new_distance = CGAL::squared_distance(pt, segments_[i]);
        if (new_distance < min_dist_sqr) {
          min_dist_sqr = new_distance;
          *best_segment = segments_[i];
        }
      }
      return min_dist_sqr;
    }
    double Distance(Point pt, Segment *best_segment) const {
      return sqrt(DistanceSqr(pt, best_segment));
    }

    bool is_inside = false;
    bool is_borderline = false;
    double dist_upper_bound_;
    double dist_lower_bound_;
    std::vector<Segment> segments_;
    std::vector<std::vector<Point>> polygons_;
  };

  class GridSystem {
   public:
    // Precision is really 2**-precision and must be positive.
    GridSystem(const std::vector<Point> &points, int precision)
        : points_(points), scale_factor_(1 << precision) {
      auto bbox = ToBbox(points);
      fprintf(stderr, "%g %g, %g %g\n", bbox.xmin(), bbox.ymin(), bbox.xmax(),
              bbox.ymax());
      x_min_ = static_cast<int>(std::floor(bbox.xmin() * scale_factor_)) - 1;
      y_min_ = static_cast<int>(std::floor(bbox.ymin() * scale_factor_)) - 1;

      stride_ = static_cast<int>(bbox.xmax() * scale_factor_) + 3 - x_min_;
      height_ = static_cast<int>(bbox.ymax() * scale_factor_) + 3 - y_min_;

      fprintf(stderr, "num_cells: %d\n", stride_ * height_);
      cells_.reserve(stride_ * height_);
      for (int y_cell = 0; y_cell < height_; ++y_cell) {
        for (int x_cell = 0; x_cell < stride_; ++x_cell) {
          cells_.push_back(
              GridCell(points, Bbox(static_cast<double>(x_cell + x_min_) /
                                        static_cast<double>(scale_factor_),
                                    static_cast<double>(y_cell + y_min_) /
                                        static_cast<double>(scale_factor_),
                                    static_cast<double>(x_cell + x_min_ + 1) /
                                        static_cast<double>(scale_factor_),
                                    static_cast<double>(y_cell + y_min_ + 1) /
                                        static_cast<double>(scale_factor_))));
        }
      }
    }

    const GridCell *GetCell(Point pt) const {
      int x_cell =
          static_cast<int>(std::floor(pt.x() * scale_factor_)) - x_min_;
      int y_cell =
          static_cast<int>(std::floor(pt.y() * scale_factor_)) - y_min_;
      if (x_cell < 0 || x_cell >= stride_) return nullptr;
      if (y_cell < 0 || y_cell >= height_) return nullptr;
      return &cells_[stride_ * y_cell + x_cell];
    }

    const std::vector<Point> &points() const { return points_; }

   private:
    std::vector<Point> points_;
    int scale_factor_;
    int x_min_;
    int y_min_;
    int stride_;
    int height_;
    std::vector<GridCell> cells_;
  };

  GridSystem grid_;
};

BoundsCheck MakeClippedArmSpace();
BoundsCheck MakeFullArmSpace();

}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTORL_LOOPS_PYTHON_ARM_BOUNDS_H_
