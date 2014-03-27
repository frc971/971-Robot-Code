#ifndef FRC971_CONTROL_LOOPS_COERCE_GOAL_H_
#define FRC971_CONTROL_LOOPS_COERCE_GOAL_H_

#include "Eigen/Dense"

#include "aos/controls/polytope.h"

namespace frc971 {
namespace control_loops {

// Intersects a line with a region, and finds the closest point to R.
// Finds a point that is closest to R inside the region, and on the line
// defined by K X = w.  If it is not possible to find a point on the line,
// finds a point that is inside the region and closest to the line.
Eigen::Matrix<double, 2, 1> CoerceGoal(aos::controls::HPolytope<2> &region,
                                       const Eigen::Matrix<double, 1, 2> &K,
                                       double w,
                                       const Eigen::Matrix<double, 2, 1> &R);

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_COERCE_GOAL_H_
