#ifndef Y2022_VISION_GEOMETRY_H_
#define Y2022_VISION_GEOMETRY_H_

#include "aos/util/math.h"
#include "glog/logging.h"
#include "opencv2/core/types.hpp"

namespace y2022::vision {

// Linear equation in the form y = mx + b
struct SlopeInterceptLine {
  double m, b;

  inline SlopeInterceptLine(cv::Point2d p, cv::Point2d q) {
    if (p.x == q.x) {
      CHECK_EQ(p.y, q.y) << "Can't fit line to infinite slope";

      // If two identical points were passed in, give the slope 0,
      // with it passing the point.
      m = 0.0;
    } else {
      m = (p.y - q.y) / (p.x - q.x);
    }
    // y = mx + b -> b = y - mx
    b = p.y - (m * p.x);
  }

  inline double operator()(double x) const { return (m * x) + b; }
};

// Linear equation in the form ax + by = c
struct StdFormLine {
 public:
  double a, b, c;

  inline std::optional<cv::Point2d> Intersection(const StdFormLine &l) const {
    // Use Cramer's rule to solve for the intersection
    const double denominator = Determinant(a, b, l.a, l.b);
    const double numerator_x = Determinant(c, b, l.c, l.b);
    const double numerator_y = Determinant(a, c, l.a, l.c);

    std::optional<cv::Point2d> intersection = std::nullopt;
    // Return nullopt if the denominator is 0, meaning the same slopes
    if (denominator != 0) {
      intersection =
          cv::Point2d(numerator_x / denominator, numerator_y / denominator);
    }

    return intersection;
  }

 private:  // Determinant of [[a, b], [c, d]]
  static inline double Determinant(double a, double b, double c, double d) {
    return (a * d) - (b * c);
  }
};

struct Circle {
 public:
  cv::Point2d center;
  double radius;

  static inline std::optional<Circle> Fit(std::vector<cv::Point2d> points) {
    CHECK_EQ(points.size(), 3ul);
    // For the 3 points, we have 3 equations in the form
    // (x - h)^2 + (y - k)^2 = r^2
    // Manipulate them to solve for the center and radius
    // (x1 - h)^2 + (y1 - k)^2 = r^2 ->
    // x1^2 + h^2 - 2x1h + y1^2 + k^2 - 2y1k = r^2
    // Also, (x2 - h)^2 + (y2 - k)^2 = r^2
    // Subtracting these two, we get
    // x1^2 - x2^2 - 2h(x1 - x2) + y1^2 - y2^2 - 2k(y1 - y2) = 0 ->
    // h(x1 - x2) + k(y1 - y2) = (-x1^2 + x2^2 - y1^2 + y2^2) / -2
    // Doing the same with equations 1 and 3, we get the second linear equation
    // h(x1 - x3) + k(y1 - y3) = (-x1^2 + x3^2 - y1^2 + y3^2) / -2
    // Now, we can solve for their intersection and find the center
    const auto l =
        StdFormLine{points[0].x - points[1].x, points[0].y - points[1].y,
                    (-std::pow(points[0].x, 2) + std::pow(points[1].x, 2) -
                     std::pow(points[0].y, 2) + std::pow(points[1].y, 2)) /
                        -2.0};
    const auto m =
        StdFormLine{points[0].x - points[2].x, points[0].y - points[2].y,
                    (-std::pow(points[0].x, 2) + std::pow(points[2].x, 2) -
                     std::pow(points[0].y, 2) + std::pow(points[2].y, 2)) /
                        -2.0};
    const auto center = l.Intersection(m);

    std::optional<Circle> circle = std::nullopt;
    if (center) {
      // Now find the radius
      const double radius = cv::norm(points[0] - *center);
      circle = Circle{*center, radius};
    }
    return circle;
  }

  inline double DistanceTo(cv::Point2d p) const {
    const auto p_prime = TranslateToOrigin(p);
    // Now, the distance is simply the difference between distance from the
    // origin to p' and the radius.
    return std::abs(cv::norm(p_prime) - radius);
  }

  inline double AngleOf(cv::Point2d p) const {
    auto p_prime = TranslateToOrigin(p);
    // Flip the y because y values go downwards.
    p_prime.y *= -1;
    return std::atan2(p_prime.y, p_prime.x);
  }

  // Expects all angles to be from 0 to 2pi
  // TODO(milind): handle wrapping
  static inline bool AngleInRange(double theta, double theta_min,
                                  double theta_max) {
    return (
        (theta >= theta_min && theta <= theta_max) ||
        (theta_min > theta_max && (theta >= theta_min || theta <= theta_max)));
  }

  inline bool InAngleRange(cv::Point2d p, double theta_min,
                           double theta_max) const {
    return AngleInRange(AngleOf(p), theta_min, theta_max);
  }

 private:
  // Translate the point on the circle
  // as if the circle's center is the origin (0,0)
  inline cv::Point2d TranslateToOrigin(cv::Point2d p) const {
    return cv::Point2d(p.x - center.x, p.y - center.y);
  }
};

}  // namespace y2022::vision

#endif  // Y2022_VISION_GEOMETRY_H_
