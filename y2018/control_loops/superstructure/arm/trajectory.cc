#include "y2018/control_loops/superstructure/arm/trajectory.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {
Path::Path(::std::initializer_list<::std::array<double, 6>> list) {
  distances_.reserve(list.size());
  ::Eigen::Matrix<double, 2, 1> last_theta;
  bool first = true;

  // Pull apart the initializer list into points, and compute the distance
  // lookup for efficient path lookups.
  for (const auto &e : list) {
    thetas_.push_back(
        (::Eigen::Matrix<double, 2, 1>() << e[0], e[1]).finished());
    omegas_.push_back(
        (::Eigen::Matrix<double, 2, 1>() << e[2], e[3]).finished());
    alphas_.push_back(
        (::Eigen::Matrix<double, 2, 1>() << e[4], e[5]).finished());

    if (first) {
      distances_.push_back(0.0);
    } else {
      distances_.push_back((thetas_.back() - last_theta).norm() +
                           distances_.back());
    }
    first = false;
    last_theta = thetas_.back();
  }
}
}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
