#include <chrono>
#include <cmath>
#include <thread>

#include <ct/optcon/optcon.h>

#include "third_party/gflags/include/gflags/gflags.h"
#include "third_party/matplotlib-cpp/matplotlibcpp.h"

DEFINE_double(boundary_scalar, 12.0, "Test command-line flag");
DEFINE_double(boundary_rate, 25.0, "Sigmoid rate");
DEFINE_bool(sigmoid, true, "If true, sigmoid, else exponential.");
DEFINE_double(round_corner, 0.0, "Corner radius of the constraint box.");

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
    derivative(0) = state(1);
    derivative(1) = control(0);
    derivative(2) = state(3);
    derivative(3) = control(1);
  }
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

  MyTermStateBarrier() {}

  MyTermStateBarrier(const MyTermStateBarrier & /*arg*/) {}

  static constexpr double kEpsilon = 1.0e-7;

  virtual ~MyTermStateBarrier() {}

  MyTermStateBarrier<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR> *clone()
      const override {
    return new MyTermStateBarrier(*this);
  }

  virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x,
                          const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> & /*u*/,
                          const SCALAR & /*t*/) override {
    SCALAR min_distance;

    // Round the corner by this amount.
    SCALAR round_corner = SCALAR(FLAGS_round_corner);

    // Positive means violation.
    SCALAR theta0_distance = x(0, 0) - (0.5 + round_corner);
    SCALAR theta1_distance = (0.8 - round_corner) - x(2, 0);
    if (theta0_distance < SCALAR(0.0) && theta1_distance < SCALAR(0.0)) {
      // Ok, both outside.  Return corner distance.
      min_distance = -hypot(theta1_distance, theta0_distance);
    } else if (theta0_distance < SCALAR(0.0) && theta1_distance > SCALAR(0.0)) {
      min_distance = theta0_distance;
    } else if (theta0_distance > SCALAR(0.0) && theta1_distance < SCALAR(0.0)) {
      min_distance = theta1_distance;
    } else {
      min_distance = ::std::min(theta0_distance, theta1_distance);
    }
    min_distance += round_corner;
    if (FLAGS_sigmoid) {
      return FLAGS_boundary_scalar /
             (1.0 + ::std::exp(-min_distance * FLAGS_boundary_rate));
    } else {
      // Values of 4 and 15 work semi resonably.
      return FLAGS_boundary_scalar *
             ::std::exp(min_distance * FLAGS_boundary_rate);
    }
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
    for (size_t i = 0; i < STATE_DIM; i += 2) {
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
};

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  // PRELIMINIARIES, see example NLOC.cpp
  constexpr size_t state_dim = MySecondOrderSystem<double>::STATE_DIM;
  constexpr size_t control_dim = MySecondOrderSystem<double>::CONTROL_DIM;

  ::std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>>
  oscillator_dynamics(new MySecondOrderSystem<double>());

  ::std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>>
      ad_linearizer(new ::ct::core::SystemLinearizer<state_dim, control_dim>(
          oscillator_dynamics));

  constexpr double kQPos = 0.5;
  constexpr double kQVel = 1.65;
  ::Eigen::Matrix<double, 4, 4> Q_step;
  Q_step << 1.0 / (kQPos * kQPos), 0.0, 0.0, 0.0, 0.0, 1.0 / (kQVel * kQVel),
      0.0, 0.0, 0.0, 0.0, 1.0 / (kQPos * kQPos), 0.0, 0.0, 0.0, 0.0,
      1.0 / (kQVel * kQVel);
  ::Eigen::Matrix<double, 2, 2> R_step;
  R_step << 1.0 / (12.0 * 12.0), 0.0, 0.0, 1.0 / (12.0 * 12.0);
  ::std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>>
      intermediate_cost(new ::ct::optcon::TermQuadratic<state_dim, control_dim>(
          Q_step, R_step));

  // TODO(austin): DARE for these.
  ::Eigen::Matrix<double, 4, 4> Q_final = Q_step;
  ::Eigen::Matrix<double, 2, 2> R_final = R_step;
  ::std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>>
      final_cost(new ::ct::optcon::TermQuadratic<state_dim, control_dim>(
          Q_final, R_final));

  ::std::shared_ptr<ct::optcon::TermBase<state_dim, control_dim>> bounds_cost(
      new MyTermStateBarrier<4, 2>());

  // TODO(austin): Cost function needs constraints.
  ::std::shared_ptr<::ct::optcon::CostFunctionQuadratic<state_dim, control_dim>>
      cost_function(
          new ::ct::optcon::CostFunctionAnalytical<state_dim, control_dim>());
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
  ::std::cout << "uub " << u_ub << ::std::endl;
  ::std::cout << "ulb " << u_lb << ::std::endl;

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
  x0 << 1.0, 0.0, 0.9, 0.0;

  constexpr ::ct::core::Time kTimeHorizon = 1.5;
  ::ct::optcon::OptConProblem<state_dim, control_dim> opt_con_problem(
      kTimeHorizon, x0, oscillator_dynamics, cost_function, ad_linearizer);
  ::ct::optcon::NLOptConSettings ilqr_settings;
  ilqr_settings.dt = 0.00505;  // the control discretization in [sec]
  ilqr_settings.integrator = ::ct::core::IntegrationType::RK4;
  ilqr_settings.discretization =
      ::ct::optcon::NLOptConSettings::APPROXIMATION::FORWARD_EULER;
  // ilqr_settings.discretization =
  //   NLOptConSettings::APPROXIMATION::MATRIX_EXPONENTIAL;
  ilqr_settings.max_iterations = 20;
  ilqr_settings.min_cost_improvement = 1.0e-11;
  ilqr_settings.nlocp_algorithm =
      ::ct::optcon::NLOptConSettings::NLOCP_ALGORITHM::ILQR;
  // the LQ-problems are solved using a custom Gauss-Newton Riccati solver
  // ilqr_settings.lqocp_solver =
  // NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;
  ilqr_settings.lqocp_solver =
      ::ct::optcon::NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;
  ilqr_settings.printSummary = true;
  if (ilqr_settings.lqocp_solver ==
      ::ct::optcon::NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER) {
    opt_con_problem.setBoxConstraints(box_constraints);
  }

  size_t K = ilqr_settings.computeK(kTimeHorizon);
  printf("Using %d steps\n", static_cast<int>(K));

  // Vector of feeback matricies.
  ::ct::core::FeedbackArray<state_dim, control_dim> u0_fb(
      K, ::ct::core::FeedbackMatrix<state_dim, control_dim>::Zero());
  ::ct::core::ControlVectorArray<control_dim> u0_ff(
      K, ::ct::core::ControlVector<control_dim>::Zero());
  ::ct::core::StateVectorArray<state_dim> x_ref_init(K + 1, x0);
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
  ilqr_settings_mpc.max_iterations = 20;
  // and we limited the printouts, too.
  ilqr_settings_mpc.printSummary = false;
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
  size_t maxNumRuns = 400;
  ::std::cout << "Starting to run MPC" << ::std::endl;

  ::std::vector<double> time_array;
  ::std::vector<double> theta1_array;
  ::std::vector<double> omega1_array;
  ::std::vector<double> theta2_array;
  ::std::vector<double> omega2_array;

  ::std::vector<double> u0_array;
  ::std::vector<double> u1_array;

  for (size_t i = 0; i < maxNumRuns; i++) {
    ::std::cout << "Solving iteration " << i << ::std::endl;
    // Time which has passed since start of MPC
    auto current_time = ::std::chrono::high_resolution_clock::now();
    ::ct::core::Time t =
        1e-6 *
        ::std::chrono::duration_cast<::std::chrono::microseconds>(current_time -
                                                                  start_time)
            .count();
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

    // TODO(austin): Re-use the policy. Maybe?  Or maybe mpc already does that.
  }
  // The summary contains some statistical data about time delays, etc.
  ilqr_mpc.printMpcSummary();

  // Now plot our simulation.
  matplotlibcpp::plot(time_array, theta1_array, {{"label", "theta1"}});
  matplotlibcpp::plot(time_array, omega1_array, {{"label", "omega1"}});
  matplotlibcpp::plot(time_array, theta2_array, {{"label", "theta2"}});
  matplotlibcpp::plot(time_array, omega2_array, {{"label", "omega2"}});
  matplotlibcpp::legend();

  matplotlibcpp::figure();
  matplotlibcpp::plot(theta1_array, theta2_array, {{"label", "trajectory"}});
  ::std::vector<double> box_x{0.5, 0.5, 1.0, 1.0};
  ::std::vector<double> box_y{0.0, 0.8, 0.8, 0.0};
  matplotlibcpp::plot(box_x, box_y, {{"label", "keepout zone"}});
  matplotlibcpp::legend();

  matplotlibcpp::figure();
  matplotlibcpp::plot(time_array, u0_array, {{"label", "u0"}});
  matplotlibcpp::plot(time_array, u1_array, {{"label", "u1"}});
  matplotlibcpp::legend();
  matplotlibcpp::show();
}
