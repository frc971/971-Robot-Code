#include "frc971/control_loops/drivetrain/distance_spline.h"

#include "aos/logging/logging.h"
#include "frc971/control_loops/drivetrain/spline.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

::std::vector<double> DistanceSpline::BuildDistances(size_t num_alpha) {
  num_alpha = num_alpha == 0 ? 100 * splines_.size() : num_alpha;
  ::std::vector<double> distances;
  distances.push_back(0.0);

  if (splines_.size() > 1) {
    // We've got a multispline to follow!
    // Confirm that the ends line up to the correct number of derivitives.
    for (size_t i = 1; i < splines_.size(); ++i) {
      const Spline &spline0 = splines_[i - 1];
      const Spline &spline1 = splines_[i];

      const ::Eigen::Matrix<double, 2, 1> end0 = spline0.Point(1.0);
      const ::Eigen::Matrix<double, 2, 1> start1 = spline1.Point(0.0);

      if (!end0.isApprox(start1, 1e-6)) {
        AOS_LOG(ERROR,
                "Splines %d and %d don't line up.  [%f, %f] != [%f, %f]\n",
                static_cast<int>(i - 1), static_cast<int>(i), end0(0, 0),
                end0(1, 0), start1(0, 0), start1(1, 0));
      }

      const ::Eigen::Matrix<double, 2, 1> dend0 = spline0.DPoint(1.0);
      const ::Eigen::Matrix<double, 2, 1> dstart1 = spline1.DPoint(0.0);

      if (!dend0.isApprox(dstart1, 1e-6)) {
        AOS_LOG(
            ERROR,
            "Splines %d and %d don't line up in the first derivitive.  [%f, "
            "%f] != [%f, %f]\n",
            static_cast<int>(i - 1), static_cast<int>(i), dend0(0, 0),
            dend0(1, 0), dstart1(0, 0), dstart1(1, 0));
      }

      const ::Eigen::Matrix<double, 2, 1> ddend0 = spline0.DDPoint(1.0);
      const ::Eigen::Matrix<double, 2, 1> ddstart1 = spline1.DDPoint(0.0);

      if (!ddend0.isApprox(ddstart1, 1e-6)) {
        AOS_LOG(
            ERROR,
            "Splines %d and %d don't line up in the second derivitive.  [%f, "
            "%f] != [%f, %f]\n",
            static_cast<int>(i - 1), static_cast<int>(i), ddend0(0, 0),
            ddend0(1, 0), ddstart1(0, 0), ddstart1(1, 0));
      }
    }
  }

  const double dalpha =
      static_cast<double>(splines_.size()) / static_cast<double>(num_alpha - 1);
  double last_alpha = 0.0;
  for (size_t i = 1; i < num_alpha; ++i) {
    const double alpha = dalpha * i;
    distances.push_back(distances.back() +
                        GaussianQuadrature5(
                            [this](double alpha) {
                              const size_t spline_index = ::std::min(
                                  static_cast<size_t>(::std::floor(alpha)),
                                  splines_.size() - 1);
                              return this->splines_[spline_index]
                                  .DPoint(alpha - spline_index)
                                  .norm();
                            },
                            last_alpha, alpha));
    last_alpha = alpha;
  }
  return distances;
}

DistanceSpline::DistanceSpline(::std::vector<Spline> &&splines, int num_alpha)
    : splines_(::std::move(splines)), distances_(BuildDistances(num_alpha)) {}

DistanceSpline::DistanceSpline(const Spline &spline, int num_alpha)
    : splines_({spline}), distances_(BuildDistances(num_alpha)) {}

::Eigen::Matrix<double, 2, 1> DistanceSpline::DDXY(double distance) const {
  const AlphaAndIndex a = DistanceToAlpha(distance);
  const ::Eigen::Matrix<double, 2, 1> dspline_point =
      splines_[a.index].DPoint(a.alpha);
  const ::Eigen::Matrix<double, 2, 1> ddspline_point =
      splines_[a.index].DDPoint(a.alpha);

  const double squared_norm = dspline_point.squaredNorm();

  return ddspline_point / squared_norm -
         dspline_point * (dspline_point(0) * ddspline_point(0) +
                          dspline_point(1) * ddspline_point(1)) /
             ::std::pow(squared_norm, 2);
}

double DistanceSpline::DDTheta(double distance) const {
  const AlphaAndIndex a = DistanceToAlpha(distance);

  // TODO(austin): We are re-computing DPoint here even worse
  const ::Eigen::Matrix<double, 2, 1> dspline_point =
      splines_[a.index].DPoint(a.alpha);
  const ::Eigen::Matrix<double, 2, 1> ddspline_point =
      splines_[a.index].DDPoint(a.alpha);

  const double dtheta = splines_[a.index].DTheta(a.alpha);
  const double ddtheta = splines_[a.index].DDTheta(a.alpha);

  const double squared_norm = dspline_point.squaredNorm();

  return ddtheta / squared_norm -
         dtheta * (dspline_point(0) * ddspline_point(0) +
                   dspline_point(1) * ddspline_point(1)) /
             ::std::pow(squared_norm, 2);
}

DistanceSpline::AlphaAndIndex DistanceSpline::DistanceToAlpha(
    double distance) const {
  if (distance <= 0.0) {
    return {0, 0.0};
  }
  if (distance >= length()) {
    return {splines_.size() - 1, 1.0};
  }

  // Find the distance right below our number using a binary search.
  size_t after = ::std::distance(
      distances_.begin(),
      ::std::lower_bound(distances_.begin(), distances_.end(), distance));
  size_t before = after - 1;
  const double distance_step_size =
      (splines_.size() / static_cast<double>(distances_.size() - 1));

  const double alpha = (distance - distances_[before]) /
                           (distances_[after] - distances_[before]) *
                           distance_step_size +
                       static_cast<double>(before) * distance_step_size;
  const size_t index = static_cast<size_t>(::std::floor(alpha));

  return {index, alpha - index};
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
