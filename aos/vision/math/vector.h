#ifndef AOS_VISION_MATH_VECTOR_H_
#define AOS_VISION_MATH_VECTOR_H_

#include <cmath>

#include "Eigen/Dense"

namespace aos {
namespace vision {

// Represents an n-dimensional vector of doubles with various convenient
// shortcuts for common operations.
//
// This includes overloads of various arithmetic operators for convenience.
// Multiplication by doubles is scalar multiplication and multiplication by
// other vectors is element-wise.
//
// Accessing elements which don't exist for a given size vector, mixing sizes of
// vectors in appropriately, etc are compile-time errors.
template <int size>
class Vector {
 public:
  Vector() { data_.SetZero(); }

  Vector(double x, double y) { Set(x, y); }

  Vector(double x, double y, double z) { Set(x, y, z); }

  double Get(int index) const { return data_(index); }
  void Set(int index, double value) { data_(index) = value; }

  void Set(double x, double y) {
    static_assert(size == 2, "illegal size");
    data_(0) = x;
    data_(1) = y;
  }
  void Set(double x, double y, double z) {
    static_assert(size == 3, "illegal size");
    data_(0) = x;
    data_(1) = y;
    data_(2) = z;
  }
  void Set(double x, double y, double z, double w) {
    static_assert(size == 4, "illegal size");
    data_(0) = x;
    data_(1) = y;
    data_(2) = z;
    data_(3) = w;
  }

  double x() const {
    static_assert(size >= 1, "illegal size");
    return data_(0);
  }
  void x(double xX) {
    static_assert(size >= 1, "illegal size");
    data_(0) = xX;
  }

  double y() const {
    static_assert(size >= 2, "illegal size");
    return data_(1);
  }
  void y(double yY) {
    static_assert(size >= 2, "illegal size");
    data_(1) = yY;
  }

  double z() const {
    static_assert(size >= 3, "illegal size");
    return data_(2);
  }
  void z(double zZ) {
    static_assert(size >= 3, "illegal size");
    data_(2) = zZ;
  }

  double w() const {
    static_assert(size >= 4, "illegal size");
    return data_(3);
  }
  void w(double wW) {
    static_assert(size >= 4, "illegal size");
    data_(3) = wW;
  }

  // Fast part of length.
  double MagSqr() const { return data_.squaredNorm(); }

  // Length of the vector.
  double Mag() const { return data_.norm(); }

  // Get underlying data structure
  ::Eigen::Matrix<double, 1, size> GetData() const { return data_; }

  // Set underlying data structure
  void SetData(const ::Eigen::Matrix<double, 1, size> &other) { data_ = other; }

  Vector<size> operator+(const Vector<size> &other) const {
    Vector<size> nv = *this;
    nv += other;
    return nv;
  }
  Vector<size> operator+=(const Vector<size> &other) {
    data_ += other.data_;
    return *this;
  }

  Vector<size> operator-(const Vector<size> &other) const {
    Vector<size> nv = *this;
    nv -= other;
    return nv;
  }
  Vector<size> operator-=(const Vector<size> &other) {
    data_ -= other.data_;
    return *this;
  }

  Vector<size> operator*(double other) {
    Vector<size> nv = *this;
    nv *= other;
    return nv;
  }
  Vector<size> operator*=(double other) {
    data_ *= other;
    return *this;
  }

  Vector<size> operator*(const Vector<size> &other) const {
    Vector<size> nv = *this;
    nv *= other;
    return nv;
  }
  Vector<size> operator*=(const Vector<size> &other) {
    for (int i = 0; i < size; i++) {
      Set(i, other.Get(i) * Get(i));
    }
    return *this;
  }

  bool operator==(const Vector<size> &other) const {
    for (int i = 0; i < size; i++) {
      if (Get(i) != other.Get(i)) {
        return false;
      }
    }
    return true;
  }

  double dot(const Vector<size> &other) const {
    return data_.dot(other.GetData());
  }

  Vector<size> cross(const Vector<size> &other) const {
    Vector<size> nv;
    nv.SetData(data_.cross(other.GetData()));
    return nv;
  }

  // Returns a vector in the same direction as this one with a magnitude of 1.
  Vector<size> Normalized() const {
    double mag = Mag();
    Vector<size> nv;
    for (int i = 0; i < size; i++) {
      nv.Set(i, Get(i) / mag);
    }
    return nv;
  }

  // Returns the angle between this vector and the 0 vector.
  // Only valid for 2-dimensional vectors.
  double AngleToZero() const {
    static_assert(size == 2, "illegal size");
    return ::std::atan2(y(), x());
  }

  // Return the angle between this and other.
  double AngleTo(const Vector<size> other) const {
    // cos(theta) = u.dot(v) / (u.magnitude() * v.magnitude())
    return ::std::acos(dot(other) / (Mag() * other.Mag()));
  }

  // Returns the distance between this and other squared.
  double SquaredDistanceTo(const Vector<size> other) {
    Vector<size> tmp = *this - other;
    return tmp.MagSqr();
  }

 private:
  // The actual data.
  ::Eigen::Matrix<double, 1, size> data_;
};

// Returns the cross product of two points.
inline double PointsCrossProduct(const Vector<2> &a, const Vector<2> &b) {
  return a.x() * b.y() - a.y() * b.x();
}

}  // namespace vision
}  // namespace aos

#endif  // AOS_VISION_MATH_VECTOR_H_
