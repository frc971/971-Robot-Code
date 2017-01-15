#ifndef AOS_VISION_COMP_GEO_SEGMENT_H_
#define AOS_VISION_COMP_GEO_SEGMENT_H_

#include <cmath>

#include "aos/vision/math/vector.h"

namespace aos {
namespace vision {

template <int Size>
class Segment {
 public:
  Segment() : A_(), B_() {}

  Segment(Vector<Size> A, Vector<Size> B) : A_(A), B_(B) {}

  Segment(double ax, double ay, double az, double bx, double by, double bz)
      : A_(ax, ay, az), B_(bx, by, bz) {}

  ~Segment() {}

  void Set(Vector<Size> a, Vector<Size> b) {
    A_ = a;
    B_ = b;
  }

  Vector<Size> A() const { return A_; }
  Vector<Size> B() const { return B_; }

  Vector<Size> AsVector() const { return B_ - A_; }
  Vector<Size> AsNormed() const { return AsVector().normed(); }

  Vector<Size> Center() const { return (A_ + B_) * (0.5); }

  // Fast part of length.
  double MagSqr() { return (AsVector()).MagSqr(); }

  // Length of the vector.
  double Mag() { return std::sqrt(MagSqr()); }

  Segment<Size> Scale(const double &other) {
    return Segment<Size>(A_ * (other), B_ * (other));
  }

  // Intersect two lines in a plane.
  Vector<2> Intersect(const Segment<2> &other) {
    static_assert(Size == 2, "Only works for size == 2");
    double x1 = A_.x();
    double y1 = A_.y();
    double x2 = B_.x();
    double y2 = B_.y();

    double x3 = other.A().x();
    double y3 = other.A().y();
    double x4 = other.B().x();
    double y4 = other.B().y();

    // check wikipedia on how to intersect two lines.
    double xn =
        (x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4);
    double xd = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    double yn =
        (x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4);
    double yd = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    return Vector<2>(xn / xd, yn / yd);
  }

 private:
  // Begining point.
  Vector<Size> A_;

  // Ending point.
  Vector<Size> B_;
};

}  // namespace vision
}  // namespace aos

#endif  // AOS_VISION_COMP_GEO_VECTOR_H_
