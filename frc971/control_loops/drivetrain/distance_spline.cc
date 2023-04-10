#include "frc971/control_loops/drivetrain/distance_spline.h"

#include "aos/logging/logging.h"
#include "frc971/control_loops/drivetrain/spline.h"
#include "glog/logging.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

::std::vector<float> DistanceSpline::BuildDistances(size_t num_alpha) {
  num_alpha = num_alpha == 0 ? 100 * splines().size() : num_alpha;
  ::std::vector<float> distances;
  distances.push_back(0.0);

  if (splines().size() > 1) {
    // We've got a multispline to follow!
    // Confirm that the ends line up to the correct number of derivatives.
    for (size_t i = 1; i < splines().size(); ++i) {
      const Spline &spline0 = splines()[i - 1];
      const Spline &spline1 = splines()[i];

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
            "Splines %d and %d don't line up in the first derivative.  [%f, "
            "%f] != [%f, %f]\n",
            static_cast<int>(i - 1), static_cast<int>(i), dend0(0, 0),
            dend0(1, 0), dstart1(0, 0), dstart1(1, 0));
      }

      const ::Eigen::Matrix<double, 2, 1> ddend0 = spline0.DDPoint(1.0);
      const ::Eigen::Matrix<double, 2, 1> ddstart1 = spline1.DDPoint(0.0);

      if (!ddend0.isApprox(ddstart1, 1e-6)) {
        AOS_LOG(
            ERROR,
            "Splines %d and %d don't line up in the second derivative.  [%.7f, "
            "%.7f] != [%.7f, %.7f]\n",
            static_cast<int>(i - 1), static_cast<int>(i), ddend0(0, 0),
            ddend0(1, 0), ddstart1(0, 0), ddstart1(1, 0));
      }
    }
  }

  const double dalpha =
      static_cast<double>(splines().size()) / static_cast<double>(num_alpha - 1);
  double last_alpha = 0.0;
  for (size_t i = 1; i < num_alpha; ++i) {
    const double alpha = dalpha * i;
    distances.push_back(distances.back() +
                        GaussianQuadrature5(
                            [this](double alpha) {
                              const size_t spline_index = ::std::min(
                                  static_cast<size_t>(::std::floor(alpha)),
                                  splines().size() - 1);
                              return this->splines()[spline_index]
                                  .DPoint(alpha - spline_index)
                                  .norm();
                            },
                            last_alpha, alpha));
    last_alpha = alpha;
  }
  return distances;
}

std::vector<Spline> FlatbufferToSplines(const MultiSpline *fb) {
  CHECK_NOTNULL(fb);
  const size_t spline_count = fb->spline_count();
  CHECK_EQ(fb->spline_x()->size(), static_cast<size_t>(spline_count * 5 + 1));
  CHECK_EQ(fb->spline_y()->size(), static_cast<size_t>(spline_count * 5 + 1));
  std::vector<Spline> splines;
  for (size_t ii = 0; ii < spline_count; ++ii) {
    Eigen::Matrix<double, 2, 6> points;
    for (int jj = 0; jj < 6; ++jj) {
      points(0, jj) = fb->spline_x()->Get(ii * 5 + jj);
      points(1, jj) = fb->spline_y()->Get(ii * 5 + jj);
    }
    splines.emplace_back(Spline(points));
  }
  return splines;
}

aos::SizedArray<Spline, FinishedDistanceSpline::kMaxSplines>
SizedFlatbufferToSplines(const MultiSpline *fb) {
  CHECK_NOTNULL(fb);
  const size_t spline_count = fb->spline_count();
  CHECK_EQ(fb->spline_x()->size(), static_cast<size_t>(spline_count * 5 + 1));
  CHECK_EQ(fb->spline_y()->size(), static_cast<size_t>(spline_count * 5 + 1));
  aos::SizedArray<Spline, FinishedDistanceSpline::kMaxSplines> splines;
  for (size_t ii = 0; ii < spline_count; ++ii) {
    Eigen::Matrix<double, 2, 6> points;
    for (int jj = 0; jj < 6; ++jj) {
      points(0, jj) = fb->spline_x()->Get(ii * 5 + jj);
      points(1, jj) = fb->spline_y()->Get(ii * 5 + jj);
    }
    splines.emplace_back(Spline(points));
  }
  return splines;
}

DistanceSpline::DistanceSpline(::std::vector<Spline> &&splines, int num_alpha)
    : splines_(::std::move(splines)), distances_(BuildDistances(num_alpha)) {}

DistanceSpline::DistanceSpline(const Spline &spline, int num_alpha)
    : splines_({spline}), distances_(BuildDistances(num_alpha)) {}

DistanceSpline::DistanceSpline(const MultiSpline *fb, int num_alpha)
    : splines_(FlatbufferToSplines(fb)),
      distances_(BuildDistances(num_alpha)) {}

// TODO(james): Directly use the flatbuffer vector for accessing distances,
// rather than doing this redundant copy.
FinishedDistanceSpline::FinishedDistanceSpline(const fb::DistanceSpline &fb)
    : splines_(SizedFlatbufferToSplines(fb.spline())),
      distances_(fb.distances()->data(), fb.distances()->size()) {}

flatbuffers::Offset<fb::DistanceSpline> DistanceSplineBase::Serialize(
    flatbuffers::FlatBufferBuilder *fbb,
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Constraint>>>
        constraints) const {
  if (splines().empty()) {
    return {};
  }
  const size_t num_points = splines().size() * 5 + 1;
  float *spline_x_vector = nullptr;
  float *spline_y_vector = nullptr;
  const flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
      fbb->CreateUninitializedVector(num_points, &spline_x_vector);
  const flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
      fbb->CreateUninitializedVector(num_points, &spline_y_vector);
  CHECK_NOTNULL(spline_x_vector);
  CHECK_NOTNULL(spline_y_vector);
  spline_x_vector[0] = splines()[0].control_points()(0, 0);
  spline_y_vector[0] = splines()[0].control_points()(1, 0);
  for (size_t spline_index = 0; spline_index < splines().size();
       ++spline_index) {
    for (size_t point = 1; point < 6u; ++point) {
      spline_x_vector[spline_index * 5 + point] =
          splines()[spline_index].control_points()(0, point);
      spline_y_vector[spline_index * 5 + point] =
          splines()[spline_index].control_points()(1, point);
    }
  }
  MultiSpline::Builder multi_spline_builder(*fbb);
  multi_spline_builder.add_spline_count(splines().size());
  multi_spline_builder.add_spline_x(spline_x_offset);
  multi_spline_builder.add_spline_y(spline_y_offset);
  multi_spline_builder.add_constraints(constraints);
  const flatbuffers::Offset<MultiSpline> multi_spline_offset =
      multi_spline_builder.Finish();
  const flatbuffers::Offset<flatbuffers::Vector<float>> distances_offset =
      fbb->CreateVector(distances().data(), distances().size());
  fb::DistanceSpline::Builder spline_builder(*fbb);
  spline_builder.add_spline(multi_spline_offset);
  spline_builder.add_distances(distances_offset);
  return spline_builder.Finish();
}

::Eigen::Matrix<double, 2, 1> DistanceSplineBase::DDXY(double distance) const {
  const AlphaAndIndex a = DistanceToAlpha(distance);
  const ::Eigen::Matrix<double, 2, 1> dspline_point =
      splines()[a.index].DPoint(a.alpha);
  const ::Eigen::Matrix<double, 2, 1> ddspline_point =
      splines()[a.index].DDPoint(a.alpha);

  const double squared_norm = dspline_point.squaredNorm();

  return ddspline_point / squared_norm -
         dspline_point *
             (dspline_point(0) * ddspline_point(0) +
              dspline_point(1) * ddspline_point(1)) /
             ::std::pow(squared_norm, 2);
}

double DistanceSplineBase::DDTheta(double distance) const {
  const AlphaAndIndex a = DistanceToAlpha(distance);

  // TODO(austin): We are re-computing DPoint here even worse
  const ::Eigen::Matrix<double, 2, 1> dspline_point =
      splines()[a.index].DPoint(a.alpha);
  const ::Eigen::Matrix<double, 2, 1> ddspline_point =
      splines()[a.index].DDPoint(a.alpha);

  const double dtheta = splines()[a.index].DTheta(a.alpha);
  const double ddtheta = splines()[a.index].DDTheta(a.alpha);

  const double squared_norm = dspline_point.squaredNorm();

  return ddtheta / squared_norm - dtheta *
                                      (dspline_point(0) * ddspline_point(0) +
                                       dspline_point(1) * ddspline_point(1)) /
                                      ::std::pow(squared_norm, 2);
}

DistanceSplineBase::AlphaAndIndex DistanceSplineBase::DistanceToAlpha(
    double distance) const {
  if (distance <= 0.0) {
    return {0, 0.0};
  }
  if (distance >= length()) {
    return {splines().size() - 1, 1.0};
  }

  // Find the distance right below our number using a binary search.
  size_t after = ::std::distance(
      distances().begin(),
      ::std::lower_bound(distances().begin(), distances().end(), distance));
  size_t before = after - 1;
  const double distance_step_size =
      (splines().size() / static_cast<double>(distances().size() - 1));

  const double alpha = (distance - distances()[before]) /
                           (distances()[after] - distances()[before]) *
                           distance_step_size +
                       static_cast<double>(before) * distance_step_size;
  const size_t index = static_cast<size_t>(::std::floor(alpha));

  return {index, alpha - index};
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
