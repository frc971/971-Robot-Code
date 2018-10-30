#include <chrono>
#include <cmath>
#include <thread>

#include <ct/optcon/optcon.h>
#include <Eigen/Eigenvalues>

#include "gflags/gflags.h"
#include "third_party/matplotlib-cpp/matplotlibcpp.h"
#include "y2018/control_loops/python/arm_bounds.h"
#include "y2018/control_loops/python/dlqr.h"

DEFINE_double(boundary_scalar, 1500.0, "Test command-line flag");
DEFINE_double(velocity_boundary_scalar, 10.0, "Test command-line flag");
DEFINE_double(boundary_rate, 20.0, "Sigmoid rate");
DEFINE_bool(linear, false, "If true, linear, else see sigmoid.");
DEFINE_bool(sigmoid, false, "If true, sigmoid, else exponential.");
DEFINE_double(round_corner, 0.0, "Corner radius of the constraint box.");
DEFINE_double(convergance, 1e-12, "Residual before finishing the solver.");
DEFINE_double(position_allowance, 5.0,
              "Distance to Velocity at which we have 0 penalty conversion.");
DEFINE_double(bounds_offset, 0.02, "Offset the quadratic boundary in by this");
DEFINE_double(linear_bounds_offset, 0.00,
              "Offset the linear boundary in by this");
DEFINE_double(yrange, 1.0,
              "+- y max for saturating out the state for the cost function.");
DEFINE_bool(debug_print, false, "Print the debugging print from the solver.");
DEFINE_bool(print_starting_summary, true,
            "Print the summary on the pre-solution.");
DEFINE_bool(print_summary, false, "Print the summary on each iteration.");
DEFINE_bool(quadratic, true, "If true, quadratic bounds penalty.");

DEFINE_bool(reset_every_cycle, false,
            "If true, reset the initial guess every cycle.");

DEFINE_double(seconds, 1.5, "The number of seconds to simulate.");

DEFINE_double(theta0, 1.0, "Starting theta0");
DEFINE_double(theta1, 0.9, "Starting theta1");

DEFINE_double(goal_theta0, -0.5, "Starting theta0");
DEFINE_double(goal_theta1, -0.5, "Starting theta1");

DEFINE_double(qpos1, 0.2, "qpos1");
DEFINE_double(qvel1, 4.0, "qvel1");
DEFINE_double(qpos2, 0.2, "qpos2");
DEFINE_double(qvel2, 4.0, "qvel2");

DEFINE_double(u_over_linear, 0.0, "Linear penalty for too much U.");
DEFINE_double(u_over_quadratic, 4.0, "Quadratic penalty for too much U.");

DEFINE_double(time_horizon, 0.75, "MPC time horizon");

DEFINE_bool(only_print_eigenvalues, false,
            "If true, stop after computing the final eigenvalues");

DEFINE_bool(plot_xy, false, "If true, plot the xy trajectory of the end of the arm.");
DEFINE_bool(plot_cost, false, "If true, plot the cost function.");
DEFINE_bool(plot_state_cost, false,
            "If true, plot the state portion of the cost function.");
DEFINE_bool(plot_states, false, "If true, plot the states.");
DEFINE_bool(plot_u, false, "If true, plot the control signal.");

static constexpr double kDt = 0.00505;

namespace y2018 {
namespace control_loops {

::Eigen::Matrix<double, 4, 4> NumericalJacobianX(
    ::Eigen::Matrix<double, 4, 1> (*fn)(
        ::Eigen::Ref<::Eigen::Matrix<double, 4, 1>> X,
        ::Eigen::Ref<::Eigen::Matrix<double, 2, 1>> U, double dt),
    ::Eigen::Matrix<double, 4, 1> X, ::Eigen::Matrix<double, 2, 1> U, double dt,
    const double kEpsilon = 1e-4) {
  constexpr int num_states = 4;
  ::Eigen::Matrix<double, 4, 4> answer = ::Eigen::Matrix<double, 4, 4>::Zero();

  // It's more expensive, but +- epsilon will be more reliable
  for (int i = 0; i < num_states; ++i) {
    ::Eigen::Matrix<double, 4, 1> dX_plus = X;
    dX_plus(i, 0) += kEpsilon;
    ::Eigen::Matrix<double, 4, 1> dX_minus = X;
    dX_minus(i, 0) -= kEpsilon;
    answer.block<4, 1>(0, i) =
        (fn(dX_plus, U, dt) - fn(dX_minus, U, dt)) / kEpsilon / 2.0;
  }
  return answer;
}

::Eigen::Matrix<double, 4, 2> NumericalJacobianU(
    ::Eigen::Matrix<double, 4, 1> (*fn)(
        ::Eigen::Ref<::Eigen::Matrix<double, 4, 1>> X,
        ::Eigen::Ref<::Eigen::Matrix<double, 2, 1>> U, double dt),
    ::Eigen::Matrix<double, 4, 1> X, ::Eigen::Matrix<double, 2, 1> U, double dt,
    const double kEpsilon = 1e-4) {
  constexpr int num_states = 4;
  constexpr int num_inputs = 2;
  ::Eigen::Matrix<double, num_states, num_inputs> answer =
      ::Eigen::Matrix<double, num_states, num_inputs>::Zero();

  // It's more expensive, but +- epsilon will be more reliable
  for (int i = 0; i < num_inputs; ++i) {
    ::Eigen::Matrix<double, 2, 1> dU_plus = U;
    dU_plus(i, 0) += kEpsilon;
    ::Eigen::Matrix<double, 2, 1> dU_minus = U;
    dU_minus(i, 0) -= kEpsilon;
    answer.block<4, 1>(0, i) =
        (fn(X, dU_plus, dt) - fn(X, dU_minus, dt)) / kEpsilon / 2.0;
  }
  return answer;
}

// This code is for analysis and simulation of a double jointed arm.  It is an
// attempt to see if a MPC could work for arm control under constraints.

// Describes a double jointed arm.
// A large chunk of this code comes from demos.  Most of the raw pointer,
// shared_ptr, and non-const &'s come from the library's conventions.
template <typename SCALAR>
class MySecondOrderSystem : public ::ct::core::ControlledSystem<4, 2, SCALAR> {
 public:
  static const size_t STATE_DIM = 4;
  static const size_t CONTROL_DIM = 2;

  MySecondOrderSystem(::std::shared_ptr<::ct::core::Controller<4, 2, SCALAR>>
                          controller = nullptr)
      : ::ct::core::ControlledSystem<4, 2, SCALAR>(
            controller, ::ct::core::SYSTEM_TYPE::GENERAL) {}

  MySecondOrderSystem(const MySecondOrderSystem &arg)
      : ::ct::core::ControlledSystem<4, 2, SCALAR>(arg) {}

  // Deep copy
  MySecondOrderSystem *clone() const override {
    return new MySecondOrderSystem(*this);
  }
  virtual ~MySecondOrderSystem() {}

  static constexpr SCALAR l1 = 46.25 * 0.0254;
  static constexpr SCALAR l2 = 41.80 * 0.0254;

  static constexpr SCALAR m1 = 9.34 / 2.2;
  static constexpr SCALAR m2 = 9.77 / 2.2;

  static constexpr SCALAR J1 = 2957.05 * 0.0002932545454545454;
  static constexpr SCALAR J2 = 2824.70 * 0.0002932545454545454;

  static constexpr SCALAR r1 = 21.64 * 0.0254;
  static constexpr SCALAR r2 = 26.70 * 0.0254;

  static constexpr SCALAR G1 = 140.0;
  static constexpr SCALAR G2 = 90.0;

  static constexpr SCALAR stall_torque = 1.41;
  static constexpr SCALAR free_speed = (5840.0 / 60.0) * 2.0 * M_PI;
  static constexpr SCALAR stall_current = 89.0;
  static constexpr SCALAR R = 12.0 / stall_current;

  static constexpr SCALAR Kv = free_speed / 12.0;
  static constexpr SCALAR Kt = stall_torque / stall_current;

  // Evaluate the system dynamics.
  //
  // Args:
  //   state: current state (position, velocity)
  //   t: current time (gets ignored)
  //   control: control action
  //   derivative: (velocity, acceleration)
  virtual void computeControlledDynamics(
      const ::ct::core::StateVector<4, SCALAR> &state, const SCALAR & /*t*/,
      const ::ct::core::ControlVector<2, SCALAR> &control,
      ::ct::core::StateVector<4, SCALAR> &derivative) override {
    derivative = Dynamics(state, control);
  }

  static ::Eigen::Matrix<double, 4, 1> Dynamics(
      const ::ct::core::StateVector<4, SCALAR> &X,
      const ::ct::core::ControlVector<2, SCALAR> &U) {
    ::ct::core::StateVector<4, SCALAR> derivative;
    const SCALAR alpha = J1 + r1 * r1 * m1 + l1 * l1 * m2;
    const SCALAR beta = l1 * r2 * m2;
    const SCALAR gamma = J2 + r2 * r2 * m2;

    const SCALAR s = sin(X(0) - X(2));
    const SCALAR c = cos(X(0) - X(2));

    // K1 * d^2 theta/dt^2 + K2 * d theta/dt = torque
    ::Eigen::Matrix<SCALAR, 2, 2> K1;
    K1(0, 0) = alpha;
    K1(1, 0) = K1(0, 1) = c * beta;
    K1(1, 1) = gamma;

    ::Eigen::Matrix<SCALAR, 2, 2> K2 = ::Eigen::Matrix<SCALAR, 2, 2>::Zero();
    K2(0, 1) = s * beta * X(3);
    K2(1, 0) = -s * beta * X(1);

    const SCALAR kNumDistalMotors = 2.0;
    ::Eigen::Matrix<SCALAR, 2, 1> torque;
    torque(0, 0) = G1 * (U(0) * Kt / R - X(1) * G1 * Kt / (Kv * R));
    torque(1, 0) = G2 * (U(1) * kNumDistalMotors * Kt / R -
                         X(3) * G2 * Kt * kNumDistalMotors / (Kv * R));

    ::Eigen::Matrix<SCALAR, 2, 1> velocity;
    velocity(0, 0) = X(0);
    velocity(1, 0) = X(2);

    const ::Eigen::Matrix<SCALAR, 2, 1> accel =
        K1.inverse() * (torque - K2 * velocity);

    derivative(0) = X(1);
    derivative(1) = accel(0);
    derivative(2) = X(3);
    derivative(3) = accel(1);

    return derivative;
  }

  // Runge-Kutta.
  static ::Eigen::Matrix<double, 4, 1> DiscreteDynamics(
      ::Eigen::Ref<::Eigen::Matrix<double, 4, 1>> X,
      ::Eigen::Ref<::Eigen::Matrix<double, 2, 1>> U, double dt) {
    const double half_dt = dt * 0.5;
    ::Eigen::Matrix<double, 4, 1> k1 = Dynamics(X, U);
    ::Eigen::Matrix<double, 4, 1> k2 = Dynamics(X + half_dt * k1, U);
    ::Eigen::Matrix<double, 4, 1> k3 = Dynamics(X + half_dt * k2, U);
    ::Eigen::Matrix<double, 4, 1> k4 = Dynamics(X + dt * k3, U);
    return X + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }
};

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL = double,
          typename SCALAR = SCALAR_EVAL>
class ObstacleAwareQuadraticCost
    : public ::ct::optcon::TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL,
                                    SCALAR> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1> state_vector_t;
  typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_t;
  typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
  typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM>
      control_state_matrix_t;
  typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM>
      state_matrix_double_t;
  typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM>
      control_matrix_double_t;
  typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM>
      control_state_matrix_double_t;

  ObstacleAwareQuadraticCost(const ::Eigen::Matrix<double, 2, 2> &R,
                             const ::Eigen::Matrix<double, 4, 4> &Q)
      : R_(R), Q_(Q) {}

  ObstacleAwareQuadraticCost(const ObstacleAwareQuadraticCost &arg)
      : R_(arg.R_), Q_(arg.Q_) {}
      static constexpr double kEpsilon = 1.0e-5;

  virtual ~ObstacleAwareQuadraticCost() {}

  ObstacleAwareQuadraticCost<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>
      *clone() const override {
    return new ObstacleAwareQuadraticCost(*this);
  }

  double SaturateX(double x, double yrange) {
    return 2.0 * ((1.0 / (1.0 + ::std::exp(-x * 2.0 / yrange)) - 0.5)) * yrange;
  }

  SCALAR distance(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x,
                  const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> & /*u*/) {
    constexpr double kCornerNewUpper0 = 0.35;
    // constexpr double kCornerUpper1 = 3.13;
    // Push it up a bit further (non-real) until we have an actual path cost.
    constexpr double kCornerNewUpper1 = 3.39;
    constexpr double kCornerNewUpper0_far = 10.0;

    // Push it up a bit further (non-real) until we have an actual path cost.
    // constexpr double kCornerUpper0 = 0.315;
    constexpr double kCornerUpper0 = 0.310;
    // constexpr double kCornerUpper1 = 3.13;
    constexpr double kCornerUpper1 = 3.25;
    constexpr double kCornerUpper0_far = 10.0;

    constexpr double kCornerLower0 = 0.023;
    constexpr double kCornerLower1 = 1.57;
    constexpr double kCornerLower0_far = 10.0;

    const Segment new_upper_segment(
        Point(kCornerNewUpper0, kCornerNewUpper1),
        Point(kCornerNewUpper0_far, kCornerNewUpper1));
    const Segment upper_segment(Point(kCornerUpper0, kCornerUpper1),
                                Point(kCornerUpper0_far, kCornerUpper1));
    const Segment lower_segment(Point(kCornerLower0, kCornerLower1),
                                Point(kCornerLower0_far, kCornerLower1));

    Point current_point(x(0, 0), x(2, 0));

    SCALAR result = 0.0;
    if (intersects(new_upper_segment,
                   Segment(current_point,
                           Point(FLAGS_goal_theta0, FLAGS_goal_theta1)))) {
      result += hypot(current_point.x() - kCornerNewUpper0,
                      current_point.y() - kCornerNewUpper1);
      current_point = Point(kCornerNewUpper0, kCornerNewUpper1);
    }

    if (intersects(upper_segment,
                   Segment(current_point,
                           Point(FLAGS_goal_theta0, FLAGS_goal_theta1)))) {
      result += hypot(current_point.x() - kCornerUpper0,
                      current_point.y() - kCornerUpper1);
      current_point = Point(kCornerUpper0, kCornerUpper1);
    }

    if (intersects(lower_segment,
                   Segment(current_point,
                           Point(FLAGS_goal_theta0, FLAGS_goal_theta1)))) {
      result += hypot(current_point.x() - kCornerLower0,
                      current_point.y() - kCornerLower1);
      current_point = Point(kCornerLower0, kCornerLower1);
    }
    result += hypot(current_point.x() - FLAGS_goal_theta0,
                    current_point.y() - FLAGS_goal_theta1);
    return result;
  }

  virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x,
                          const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u,
                          const SCALAR & /*t*/) override {
    // Positive means violation.
    Eigen::Matrix<SCALAR, STATE_DIM, 1> saturated_x = x;
    SCALAR d = distance(x, u);
    saturated_x(0, 0) = d;
    saturated_x(2, 0) = 0.0;

    saturated_x(0, 0) = SaturateX(saturated_x(0, 0), FLAGS_yrange);
    saturated_x(2, 0) = 0.0;

    //SCALAR saturation_scalar = saturated_x(0, 0) / d;
    //saturated_x(1, 0) *= saturation_scalar;
    //saturated_x(3, 0) *= saturation_scalar;

    SCALAR result = (saturated_x.transpose() * Q_ * saturated_x +
                     u.transpose() * R_ * u)(0, 0);

    if (::std::abs(u(0, 0)) > 11.0) {
      result += (::std::abs(u(0, 0)) - 11.0) * FLAGS_u_over_linear;
      result += (::std::abs(u(0, 0)) - 11.0) * (::std::abs(u(0, 0)) - 11.0) *
                FLAGS_u_over_quadratic;
    }
    if (::std::abs(u(1, 0)) > 11.0) {
      result += (::std::abs(u(1, 0)) - 11.0) * FLAGS_u_over_linear;
      result += (::std::abs(u(1, 0)) - 11.0) * (::std::abs(u(1, 0)) - 11.0) *
                FLAGS_u_over_quadratic;
    }
    return result;
  }

  ct::core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivative(
      const ct::core::StateVector<STATE_DIM, SCALAR_EVAL> &x,
      const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> &u,
      const SCALAR_EVAL &t) override {
    SCALAR epsilon = SCALAR(kEpsilon);

    ct::core::StateVector<STATE_DIM, SCALAR_EVAL> result =
        ct::core::StateVector<STATE_DIM, SCALAR_EVAL>::Zero();

    // Perterb x for both position axis and return the result.
    for (size_t i = 0; i < STATE_DIM; i += 1) {
      ct::core::StateVector<STATE_DIM, SCALAR_EVAL> plus_perterbed_x = x;
      ct::core::StateVector<STATE_DIM, SCALAR_EVAL> minus_perterbed_x = x;
      plus_perterbed_x[i] += epsilon;
      minus_perterbed_x[i] -= epsilon;
      result[i] = (evaluate(plus_perterbed_x, u, t) -
                   evaluate(minus_perterbed_x, u, t)) /
                  (epsilon * 2.0);
    }
    return result;
  }

  // Compute second order derivative of this cost term w.r.t. the state
  state_matrix_t stateSecondDerivative(
      const ct::core::StateVector<STATE_DIM, SCALAR_EVAL> &x,
      const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> &u,
      const SCALAR_EVAL &t) override {
    state_matrix_t result = state_matrix_t::Zero();

    SCALAR epsilon = SCALAR(kEpsilon);

    // Perterb x a second time.
    for (size_t i = 0; i < STATE_DIM; i += 1) {
      ct::core::StateVector<STATE_DIM, SCALAR_EVAL> plus_perterbed_x = x;
      ct::core::StateVector<STATE_DIM, SCALAR_EVAL> minus_perterbed_x = x;
      plus_perterbed_x[i] += epsilon;
      minus_perterbed_x[i] -= epsilon;
      state_vector_t delta = (stateDerivative(plus_perterbed_x, u, t) -
                              stateDerivative(minus_perterbed_x, u, t)) /
                             (epsilon * 2.0);

      result.col(i) = delta;
    }
    //::std::cout << "Q_numeric " << result << " endQ" << ::std::endl;
    return result;
  }

  ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> controlDerivative(
      const ct::core::StateVector<STATE_DIM, SCALAR_EVAL> &x,
      const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> &u,
      const SCALAR_EVAL &t) override {
    ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> result =
        ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL>::Zero();

    SCALAR epsilon = SCALAR(kEpsilon);

    // Perterb x a second time.
    for (size_t i = 0; i < CONTROL_DIM; i += 1) {
      ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> plus_perterbed_u = u;
      ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> minus_perterbed_u = u;
      plus_perterbed_u[i] += epsilon;
      minus_perterbed_u[i] -= epsilon;
      SCALAR delta = (evaluate(x, plus_perterbed_u, t) -
                      evaluate(x, minus_perterbed_u, t)) /
                     (epsilon * 2.0);

      result[i] = delta;
    }
    //::std::cout << "cd " << result(0, 0) << " " << result(1, 0) << " endcd"
                //<< ::std::endl;

    return result;
  }

  control_state_matrix_t stateControlDerivative(
      const ct::core::StateVector<STATE_DIM, SCALAR_EVAL> & /*x*/,
      const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> & /*u*/,
      const SCALAR_EVAL & /*t*/) override {
    // No coupling here, so let's not bother to calculate it.
    control_state_matrix_t result = control_state_matrix_t::Zero();
    return result;
  }

  control_matrix_t controlSecondDerivative(
      const ct::core::StateVector<STATE_DIM, SCALAR_EVAL> &x,
      const ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> &u,
      const SCALAR_EVAL &t) override {
    control_matrix_t result = control_matrix_t::Zero();

    SCALAR epsilon = SCALAR(kEpsilon);

    //static int j = 0;
    //::std::this_thread::sleep_for(::std::chrono::milliseconds(j % 10));
    //int k = ++j;
    // Perterb x a second time.
    for (size_t i = 0; i < CONTROL_DIM; i += 1) {
      ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> plus_perterbed_u = u;
      ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> minus_perterbed_u = u;
      plus_perterbed_u[i] += epsilon;
      minus_perterbed_u[i] -= epsilon;
      ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> delta =
          (controlDerivative(x, plus_perterbed_u, t) -
           controlDerivative(x, minus_perterbed_u, t)) /
          (epsilon * 2.0);

      //::std::cout << "delta: " << delta(0, 0) << " " << delta(1, 0) << " k "
                  //<< k << ::std::endl;
      result.col(i) = delta;
    }
    //::std::cout << "R_numeric " << result << " endR 0.013888888888888888    k:" << k
                //<< ::std::endl;
    //::std::cout << "x " << x << " u " << u << "    k " << k << ::std::endl;

    return result;
  }

 private:
  const ::Eigen::Matrix<double, 2, 2> R_;
  const ::Eigen::Matrix<double, 4, 4> Q_;
};

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR_EVAL = double,
          typename SCALAR = SCALAR_EVAL>
class MyTermStateBarrier : public ::ct::optcon::TermBase<STATE_DIM, CONTROL_DIM,
                                                         SCALAR_EVAL, SCALAR> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, 1> state_vector_t;
  typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_t;
  typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
  typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM>
      control_state_matrix_t;
  typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM>
      state_matrix_double_t;
  typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM>
      control_matrix_double_t;
  typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM>
      control_state_matrix_double_t;

  MyTermStateBarrier(BoundsCheck *bounds_check) : bounds_check_(bounds_check) {}

  MyTermStateBarrier(const MyTermStateBarrier &arg)
      : bounds_check_(arg.bounds_check_) {}

  static constexpr double kEpsilon = 5.0e-6;

  virtual ~MyTermStateBarrier() {}

  MyTermStateBarrier<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR> *clone()
      const override {
    return new MyTermStateBarrier(*this);
  }

  SCALAR distance(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x,
                  const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> & /*u*/,
                  const SCALAR & /*t*/, Eigen::Matrix<SCALAR, 2, 1> *norm) {
    return bounds_check_->min_distance(Point(x(0, 0), x(2, 0)), norm);
  }

  virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x,
                          const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> & u,
                          const SCALAR & t) override {
    Eigen::Matrix<SCALAR, 2, 1> norm = Eigen::Matrix<SCALAR, 2, 1>::Zero();
    SCALAR min_distance = distance(x, u, t, &norm);

    // Velocity component (+) towards the wall.
    SCALAR velocity_penalty = -(x(1, 0) * norm(0, 0) + x(3, 0) * norm(1, 0));
    if (min_distance + FLAGS_bounds_offset < 0.0) {
      velocity_penalty = 0.0;
    }

    SCALAR result;
    //if (FLAGS_quadratic) {
    result = FLAGS_boundary_scalar *
                 ::std::max(0.0, min_distance + FLAGS_bounds_offset) *
                 ::std::max(0.0, min_distance + FLAGS_bounds_offset) +
             FLAGS_boundary_rate *
                 ::std::max(0.0, min_distance + FLAGS_linear_bounds_offset) +
             FLAGS_velocity_boundary_scalar *
                 ::std::max(0.0, min_distance + FLAGS_linear_bounds_offset) *
                 ::std::max(0.0, velocity_penalty) *
                 ::std::max(0.0, velocity_penalty);
    /*
} else if (FLAGS_linear) {
result =
FLAGS_boundary_scalar * ::std::max(0.0, min_distance) +
FLAGS_velocity_boundary_scalar * ::std::max(0.0, -velocity_penalty);
} else if (FLAGS_sigmoid) {
result = FLAGS_boundary_scalar /
    (1.0 + ::std::exp(-min_distance * FLAGS_boundary_rate)) +
FLAGS_velocity_boundary_scalar /
    (1.0 + ::std::exp(-velocity_penalty * FLAGS_boundary_rate));
} else {
// Values of 4 and 15 work semi resonably.
result = FLAGS_boundary_scalar *
    ::std::exp(min_distance * FLAGS_boundary_rate) +
FLAGS_velocity_boundary_scalar *
    ::std::exp(velocity_penalty * FLAGS_boundary_rate);
}
if (result < 0.0) {
printf("Result negative %f\n", result);
}
*/
    return result;
  }

  ::ct::core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivative(
      const ::ct::core::StateVector<STATE_DIM, SCALAR_EVAL> &x,
      const ::ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> &u,
      const SCALAR_EVAL &t) override {
    SCALAR epsilon = SCALAR(kEpsilon);

    ::ct::core::StateVector<STATE_DIM, SCALAR_EVAL> result =
        ::ct::core::StateVector<STATE_DIM, SCALAR_EVAL>::Zero();

    // Perturb x for both position axis and return the result.
    for (size_t i = 0; i < STATE_DIM; i += 2) {
      ::ct::core::StateVector<STATE_DIM, SCALAR_EVAL> plus_perterbed_x = x;
      ::ct::core::StateVector<STATE_DIM, SCALAR_EVAL> minus_perterbed_x = x;
      plus_perterbed_x[i] += epsilon;
      minus_perterbed_x[i] -= epsilon;
      result[i] = (evaluate(plus_perterbed_x, u, t) -
                   evaluate(minus_perterbed_x, u, t)) /
                  (epsilon * 2.0);
    }
    return result;
  }

  // Compute second order derivative of this cost term w.r.t. the state
  state_matrix_t stateSecondDerivative(
      const ::ct::core::StateVector<STATE_DIM, SCALAR_EVAL> &x,
      const ::ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> &u,
      const SCALAR_EVAL &t) override {
    state_matrix_t result = state_matrix_t::Zero();

    SCALAR epsilon = SCALAR(kEpsilon);

    // Perturb x a second time.
    for (size_t i = 0; i < STATE_DIM; i += 1) {
      ::ct::core::StateVector<STATE_DIM, SCALAR_EVAL> plus_perterbed_x = x;
      ::ct::core::StateVector<STATE_DIM, SCALAR_EVAL> minus_perterbed_x = x;
      plus_perterbed_x[i] += epsilon;
      minus_perterbed_x[i] -= epsilon;
      state_vector_t delta = (stateDerivative(plus_perterbed_x, u, t) -
                              stateDerivative(minus_perterbed_x, u, t)) /
                             (epsilon * 2.0);

      result.col(i) = delta;
    }
    return result;
  }

  ::ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> controlDerivative(
      const ::ct::core::StateVector<STATE_DIM, SCALAR_EVAL> & /*x*/,
      const ::ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> & /*u*/,
      const SCALAR_EVAL & /*t*/) override {
    return ::ct::core::StateVector<CONTROL_DIM, SCALAR_EVAL>::Zero();
  }

  control_state_matrix_t stateControlDerivative(
      const ::ct::core::StateVector<STATE_DIM, SCALAR_EVAL> & /*x*/,
      const ::ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> & /*u*/,
      const SCALAR_EVAL & /*t*/) override {
    control_state_matrix_t result = control_state_matrix_t::Zero();

    return result;
  }

  control_matrix_t controlSecondDerivative(
      const ::ct::core::StateVector<STATE_DIM, SCALAR_EVAL> & /*x*/,
      const ::ct::core::ControlVector<CONTROL_DIM, SCALAR_EVAL> & /*u*/,
      const SCALAR_EVAL & /*t*/) override {
    control_matrix_t result = control_matrix_t::Zero();
    return result;
  }

  /*
    // TODO(austin): Implement this for the automatic differentiation.
    virtual ::ct::core::ADCGScalar evaluateCppadCg(
        const ::ct::core::StateVector<STATE_DIM, ::ct::core::ADCGScalar> &x,
        const ::ct::core::ControlVector<CONTROL_DIM, ::ct::core::ADCGScalar> &u,
        ::ct::core::ADCGScalar t) override {
      ::ct::core::ADCGScalar c = ::ct::core::ADCGScalar(0.0);
      for (size_t i = 0; i < STATE_DIM; i++)
        c += barriers_[i].computeActivation(x(i));
      return c;
    }
  */

  BoundsCheck *bounds_check_;
};

int Main() {
  // PRELIMINIARIES
  BoundsCheck arm_space = MakeClippedArmSpace();

  constexpr size_t state_dim = MySecondOrderSystem<double>::STATE_DIM;
  constexpr size_t control_dim = MySecondOrderSystem<double>::CONTROL_DIM;

  ::std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>>
  oscillator_dynamics(new MySecondOrderSystem<double>());

  ::std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>>
      ad_linearizer(new ::ct::core::SystemLinearizer<state_dim, control_dim>(
          oscillator_dynamics));

  const double kQPos1 = FLAGS_qpos1;
  const double kQVel1 = FLAGS_qvel1;
  const double kQPos2 = FLAGS_qpos2;
  const double kQVel2 = FLAGS_qvel2;

  ::Eigen::Matrix<double, 4, 4> Q_step;
  Q_step << 1.0 / (kQPos1 * kQPos1), 0.0, 0.0, 0.0, 0.0,
      1.0 / (kQVel1 * kQVel1), 0.0, 0.0, 0.0, 0.0, 1.0 / (kQPos2 * kQPos2), 0.0,
      0.0, 0.0, 0.0, 1.0 / (kQVel2 * kQVel2);
  ::Eigen::Matrix<double, 2, 2> R_step;
  R_step << 1.0 / (12.0 * 12.0), 0.0, 0.0, 1.0 / (12.0 * 12.0);
  ::std::shared_ptr<::ct::optcon::TermQuadratic<state_dim, control_dim>>
      quadratic_intermediate_cost(
          new ::ct::optcon::TermQuadratic<state_dim, control_dim>(Q_step,
                                                                  R_step));
  // TODO(austin): Move back to this with the new Q and R
  ::std::shared_ptr<ObstacleAwareQuadraticCost<state_dim, control_dim>>
      intermediate_cost(new ObstacleAwareQuadraticCost<4, 2>(R_step, Q_step));

  ::Eigen::Matrix<double, 4, 4> final_A =
      NumericalJacobianX(MySecondOrderSystem<double>::DiscreteDynamics,
                         Eigen::Matrix<double, 4, 1>::Zero(),
                         Eigen::Matrix<double, 2, 1>::Zero(), kDt);

  ::Eigen::Matrix<double, 4, 2> final_B =
      NumericalJacobianU(MySecondOrderSystem<double>::DiscreteDynamics,
                         Eigen::Matrix<double, 4, 1>::Zero(),
                         Eigen::Matrix<double, 2, 1>::Zero(), kDt);

  ::Eigen::Matrix<double, 4, 4> S_lqr;
  ::Eigen::Matrix<double, 2, 4> K_lqr;
  ::frc971::controls::dlqr(final_A, final_B, Q_step, R_step, &K_lqr, &S_lqr);
  ::std::cout << "A -> " << ::std::endl << final_A << ::std::endl;
  ::std::cout << "B -> " << ::std::endl << final_B << ::std::endl;
  ::std::cout << "K -> " << ::std::endl << K_lqr << ::std::endl;
  ::std::cout << "S -> " << ::std::endl << S_lqr << ::std::endl;
  ::std::cout << "Q -> " << ::std::endl << Q_step << ::std::endl;
  ::std::cout << "R -> " << ::std::endl << R_step << ::std::endl;
  ::std::cout << "Eigenvalues: " << (final_A - final_B * K_lqr).eigenvalues()
              << ::std::endl;

  ::Eigen::Matrix<double, 4, 4> Q_final = 0.5 * S_lqr;
  ::Eigen::Matrix<double, 2, 2> R_final = ::Eigen::Matrix<double, 2, 2>::Zero();
  ::std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>>
      final_cost(new ::ct::optcon::TermQuadratic<state_dim, control_dim>(
          Q_final, R_final));
  if (FLAGS_only_print_eigenvalues) {
    return 0;
  }

  ::std::shared_ptr<MyTermStateBarrier<state_dim, control_dim>> bounds_cost(
      new MyTermStateBarrier<4, 2>(&arm_space));

  // TODO(austin): Cost function needs constraints.
  ::std::shared_ptr<::ct::optcon::CostFunctionQuadratic<state_dim, control_dim>>
      cost_function(
          new ::ct::optcon::CostFunctionAnalytical<state_dim, control_dim>());
  //cost_function->addIntermediateTerm(quadratic_intermediate_cost);
  cost_function->addIntermediateTerm(intermediate_cost);
  cost_function->addIntermediateTerm(bounds_cost);
  cost_function->addFinalTerm(final_cost);

  // STEP 1-D: set up the box constraints for the control input
  // input box constraint boundaries with sparsities in constraint toolbox
  // format
  Eigen::VectorXd u_lb(control_dim);
  Eigen::VectorXd u_ub(control_dim);
  u_ub.setConstant(12.0);
  u_lb = -u_ub;
  //::std::cout << "uub " << u_ub << ::std::endl;
  //::std::cout << "ulb " << u_lb << ::std::endl;

  // constraint terms
  std::shared_ptr<::ct::optcon::ControlInputConstraint<state_dim, control_dim>>
      controlConstraint(
          new ::ct::optcon::ControlInputConstraint<state_dim, control_dim>(
              u_lb, u_ub));
  controlConstraint->setName("ControlInputConstraint");
  // create constraint container
  std::shared_ptr<
      ::ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>>
      box_constraints(
          new ::ct::optcon::ConstraintContainerAnalytical<state_dim,
                                                          control_dim>());
  // add and initialize constraint terms
  box_constraints->addIntermediateConstraint(controlConstraint, true);
  box_constraints->initialize();

  // Starting point.
  ::ct::core::StateVector<state_dim> x0;
  x0 << FLAGS_theta0, 0.0, FLAGS_theta1, 0.0;

  const ::ct::core::Time kTimeHorizon = FLAGS_time_horizon;
  ::ct::optcon::OptConProblem<state_dim, control_dim> opt_con_problem(
      kTimeHorizon, x0, oscillator_dynamics, cost_function, ad_linearizer);
  ::ct::optcon::NLOptConSettings ilqr_settings;
  ilqr_settings.nThreads = 4;
  ilqr_settings.dt = kDt;  // the control discretization in [sec]
  ilqr_settings.integrator = ::ct::core::IntegrationType::RK4;
  ilqr_settings.debugPrint = FLAGS_debug_print;
  ilqr_settings.discretization =
      ::ct::optcon::NLOptConSettings::APPROXIMATION::FORWARD_EULER;
  // ilqr_settings.discretization =
  //   NLOptConSettings::APPROXIMATION::MATRIX_EXPONENTIAL;
  ilqr_settings.max_iterations = 40;
  ilqr_settings.min_cost_improvement = FLAGS_convergance;
  ilqr_settings.nlocp_algorithm =
      //::ct::optcon::NLOptConSettings::NLOCP_ALGORITHM::ILQR;
      ::ct::optcon::NLOptConSettings::NLOCP_ALGORITHM::GNMS;
  // the LQ-problems are solved using a custom Gauss-Newton Riccati solver
  ilqr_settings.lqocp_solver =
      ::ct::optcon::NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
  //ilqr_settings.lqocp_solver =
      //::ct::optcon::NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;
  ilqr_settings.printSummary = FLAGS_print_starting_summary;
  if (ilqr_settings.lqocp_solver ==
      ::ct::optcon::NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER) {
    //opt_con_problem.setBoxConstraints(box_constraints);
  }

  const size_t num_steps = ilqr_settings.computeK(kTimeHorizon);
  printf("Using %d steps\n", static_cast<int>(num_steps));

  // Vector of feeback matricies.
  ::ct::core::FeedbackArray<state_dim, control_dim> u0_fb(
      num_steps, ::ct::core::FeedbackMatrix<state_dim, control_dim>::Zero());
  ::ct::core::ControlVectorArray<control_dim> u0_ff(
      num_steps, ::ct::core::ControlVector<control_dim>::Zero());
  ::ct::core::StateVectorArray<state_dim> x_ref_init(num_steps + 1, x0);
  ::ct::core::StateFeedbackController<state_dim, control_dim>
      initial_controller(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);

  // STEP 2-C: create an NLOptConSolver instance
  ::ct::optcon::NLOptConSolver<state_dim, control_dim> iLQR(opt_con_problem,
                                                            ilqr_settings);
  // Seed it with the initial guess
  iLQR.setInitialGuess(initial_controller);
  // we solve the optimal control problem and retrieve the solution
  iLQR.solve();
  ::ct::core::StateFeedbackController<state_dim, control_dim> initial_solution =
      iLQR.getSolution();
  // MPC-EXAMPLE
  // we store the initial solution obtained from solving the initial optimal
  // control problem, and re-use it to initialize the MPC solver in the
  // following.

  // STEP 1: first, we set up an MPC instance for the iLQR solver and configure
  // it. Since the MPC class is wrapped around normal Optimal Control Solvers,
  // we need to different kind of settings, those for the optimal control
  // solver, and those specific to MPC:

  // 1) settings for the iLQR instance used in MPC. Of course, we use the same
  // settings as for solving the initial problem ...
  ::ct::optcon::NLOptConSettings ilqr_settings_mpc = ilqr_settings;
  ilqr_settings_mpc.max_iterations = 40;
  // and we limited the printouts, too.
  ilqr_settings_mpc.printSummary = FLAGS_print_summary;
  // 2) settings specific to model predictive control. For a more detailed
  // description of those, visit ct/optcon/mpc/MpcSettings.h
  ::ct::optcon::mpc_settings mpc_settings;
  mpc_settings.stateForwardIntegration_ = true;
  mpc_settings.postTruncation_ = false;
  mpc_settings.measureDelay_ = false;
  mpc_settings.fixedDelayUs_ = 5000 * 0;  // Ignore the delay for now.
  mpc_settings.delayMeasurementMultiplier_ = 1.0;
  // mpc_settings.mpc_mode = ::ct::optcon::MPC_MODE::FIXED_FINAL_TIME;
  mpc_settings.mpc_mode = ::ct::optcon::MPC_MODE::CONSTANT_RECEDING_HORIZON;
  mpc_settings.coldStart_ = false;

  // STEP 2 : Create the iLQR-MPC object, based on the optimal control problem
  // and the selected settings.
  ::ct::optcon::MPC<::ct::optcon::NLOptConSolver<state_dim, control_dim>>
      ilqr_mpc(opt_con_problem, ilqr_settings_mpc, mpc_settings);
  // initialize it using the previously computed initial controller
  ilqr_mpc.setInitialGuess(initial_solution);
  // STEP 3: running MPC
  // Here, we run the MPC loop. Note that the general underlying idea is that
  // you receive a state-estimate together with a time-stamp from your robot or
  // system. MPC needs to receive both that time information and the state from
  // your control system. Here, "simulate" the time measurement using
  // ::std::chrono and wrap everything into a for-loop.
  // The basic idea of operation is that after receiving time and state
  // information, one executes the finishIteration() method of MPC.
  ///
  auto start_time = ::std::chrono::high_resolution_clock::now();
  // limit the maximum number of runs in this example
  size_t maxNumRuns = FLAGS_seconds / kDt;
  ::std::cout << "Starting to run MPC" << ::std::endl;

  ::std::vector<double> time_array;
  ::std::vector<double> theta1_array;
  ::std::vector<double> omega1_array;
  ::std::vector<double> theta2_array;
  ::std::vector<double> omega2_array;

  ::std::vector<double> u0_array;
  ::std::vector<double> u1_array;

  ::std::vector<double> x_array;
  ::std::vector<double> y_array;

  // TODO(austin): Plot x, y of the end of the arm.

  for (size_t i = 0; i < maxNumRuns; i++) {
    ::std::cout << "Solving iteration " << i << ::std::endl;
    // Time which has passed since start of MPC
    auto current_time = ::std::chrono::high_resolution_clock::now();
    ::ct::core::Time t =
        1e-6 *
        ::std::chrono::duration_cast<::std::chrono::microseconds>(current_time -
                                                                  start_time)
            .count();
    {
      if (FLAGS_reset_every_cycle) {
        ::ct::core::FeedbackArray<state_dim, control_dim> u0_fb(
            num_steps,
            ::ct::core::FeedbackMatrix<state_dim, control_dim>::Zero());
        ::ct::core::ControlVectorArray<control_dim> u0_ff(
            num_steps, ::ct::core::ControlVector<control_dim>::Zero());
        ::ct::core::StateVectorArray<state_dim> x_ref_init(num_steps + 1, x0);
        ::ct::core::StateFeedbackController<state_dim, control_dim>
            resolved_controller(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);

        iLQR.setInitialGuess(initial_controller);
        // we solve the optimal control problem and retrieve the solution
        iLQR.solve();
        resolved_controller = iLQR.getSolution();
        ilqr_mpc.setInitialGuess(resolved_controller);
      }
    }

    // prepare mpc iteration
    ilqr_mpc.prepareIteration(t);
    // new optimal policy
    ::std::shared_ptr<ct::core::StateFeedbackController<state_dim, control_dim>>
        newPolicy(
            new ::ct::core::StateFeedbackController<state_dim, control_dim>());
    // timestamp of the new optimal policy
    ::ct::core::Time ts_newPolicy;
    current_time = ::std::chrono::high_resolution_clock::now();
    t = 1e-6 *
        ::std::chrono::duration_cast<::std::chrono::microseconds>(current_time -
                                                                  start_time)
            .count();
    // TODO(austin): This is only iterating once...  I need to fix that...
    //  NLOptConSolver::solve() runs for upto N iterations.  This call runs
    //  runIteration() effectively once.  (nlocAlgorithm_ is iLQR)
    bool success = ilqr_mpc.finishIteration(x0, t, *newPolicy, ts_newPolicy);
    // we break the loop in case the time horizon is reached or solve() failed
    if (ilqr_mpc.timeHorizonReached() | !success) break;

    ::std::cout << "Solved  for time " << newPolicy->time()[0] << " state "
                << x0.transpose() << " next time " << newPolicy->time()[1]
                << ::std::endl;
    ::std::cout << "  Solution: Uff " << newPolicy->uff()[0].transpose()
                << " x_ref_ " << newPolicy->x_ref()[0].transpose()
                << ::std::endl;

    time_array.push_back(ilqr_settings.dt * i);
    theta1_array.push_back(x0(0));
    omega1_array.push_back(x0(1));
    theta2_array.push_back(x0(2));
    omega2_array.push_back(x0(3));

    u0_array.push_back(newPolicy->uff()[0](0, 0));
    u1_array.push_back(newPolicy->uff()[0](1, 0));

    ::std::cout << "xref[1] " << newPolicy->x_ref()[1].transpose()
                << ::std::endl;
    ilqr_mpc.doForwardIntegration(0.0, ilqr_settings.dt, x0, newPolicy);
    ::std::cout << "Next X:  " << x0.transpose() << ::std::endl;

    x_array.push_back(MySecondOrderSystem<double>::l1 * sin(x0(0)) +
                      MySecondOrderSystem<double>::r2 * sin(x0(2)));
    y_array.push_back(MySecondOrderSystem<double>::l1 * cos(x0(0)) +
                      MySecondOrderSystem<double>::r2 * cos(x0(2)));

    // TODO(austin): Re-use the policy. Maybe?  Or maybe mpc already does that.
  }
  // The summary contains some statistical data about time delays, etc.
  ilqr_mpc.printMpcSummary();

  if (FLAGS_plot_states) {
    // Now plot our simulation.
    matplotlibcpp::plot(time_array, theta1_array, {{"label", "theta1"}});
    matplotlibcpp::plot(time_array, omega1_array, {{"label", "omega1"}});
    matplotlibcpp::plot(time_array, theta2_array, {{"label", "theta2"}});
    matplotlibcpp::plot(time_array, omega2_array, {{"label", "omega2"}});
    matplotlibcpp::legend();
  }

  if (FLAGS_plot_xy) {
    matplotlibcpp::figure();
    matplotlibcpp::plot(x_array, y_array, {{"label", "xy trajectory"}});
    matplotlibcpp::legend();
  }

  if (FLAGS_plot_u) {
    matplotlibcpp::figure();
    matplotlibcpp::plot(time_array, u0_array, {{"label", "u0"}});
    matplotlibcpp::plot(time_array, u1_array, {{"label", "u1"}});
    matplotlibcpp::legend();
  }

  ::std::vector<::std::vector<double>> cost_x;
  ::std::vector<::std::vector<double>> cost_y;
  ::std::vector<::std::vector<double>> cost_z;
  ::std::vector<::std::vector<double>> cost_state_z;

  for (double x_coordinate = -0.5; x_coordinate < 1.2; x_coordinate += 0.05) {
    ::std::vector<double> cost_x_row;
    ::std::vector<double> cost_y_row;
    ::std::vector<double> cost_z_row;
    ::std::vector<double> cost_state_z_row;

    for (double y_coordinate = -1.0; y_coordinate < 6.0; y_coordinate += 0.05) {
      cost_x_row.push_back(x_coordinate);
      cost_y_row.push_back(y_coordinate);
      Eigen::Matrix<double, 4, 1> state_matrix;
      state_matrix << x_coordinate, 0.0, y_coordinate, 0.0;
      Eigen::Matrix<double, 2, 1> u_matrix =
          Eigen::Matrix<double, 2, 1>::Zero();
      cost_state_z_row.push_back(
          intermediate_cost->distance(state_matrix, u_matrix));
      cost_z_row.push_back(
          ::std::min(bounds_cost->evaluate(state_matrix, u_matrix, 0.0), 50.0));
    }
    cost_x.push_back(cost_x_row);
    cost_y.push_back(cost_y_row);
    cost_z.push_back(cost_z_row);
    cost_state_z.push_back(cost_state_z_row);
  }

  if (FLAGS_plot_cost) {
    matplotlibcpp::plot_surface(cost_x, cost_y, cost_z);
  }

  if (FLAGS_plot_state_cost) {
    matplotlibcpp::plot_surface(cost_x, cost_y, cost_state_z);
  }

  matplotlibcpp::figure();
  matplotlibcpp::plot(theta1_array, theta2_array, {{"label", "trajectory"}});
  ::std::vector<double> bounds_x;
  ::std::vector<double> bounds_y;
  for (const Point p : arm_space.points()) {
    bounds_x.push_back(p.x());
    bounds_y.push_back(p.y());
  }
  matplotlibcpp::plot(bounds_x, bounds_y, {{"label", "allowed region"}});
  matplotlibcpp::legend();

  matplotlibcpp::show();

  return 0;
}

}  // namespace control_loops
}  // namespace y2018

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  return ::y2018::control_loops::Main();
}
