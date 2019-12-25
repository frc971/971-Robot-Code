#include "y2019/control_loops/drivetrain/localizer.h"

#include <queue>
#include <random>

#include "aos/testing/random_seed.h"
#include "aos/testing/test_shm.h"
#include "frc971/control_loops/drivetrain/splinedrivetrain.h"
#include "frc971/control_loops/drivetrain/trajectory.h"
#include "gflags/gflags.h"
#if defined(SUPPORT_PLOT)
#include "third_party/matplotlib-cpp/matplotlibcpp.h"
#endif
#include "gtest/gtest.h"
#include "y2019/constants.h"
#include "y2019/control_loops/drivetrain/drivetrain_base.h"

DEFINE_bool(plot, false, "If true, plot");

namespace y2019 {
namespace control_loops {
namespace testing {

using ::y2019::constants::Field;

constexpr size_t kNumCameras = 4;
constexpr size_t kNumTargetsPerFrame = 3;

typedef TypedLocalizer<kNumCameras, Field::kNumTargets, Field::kNumObstacles,
                       kNumTargetsPerFrame, double>
    TestLocalizer;
typedef typename TestLocalizer::Camera TestCamera;
typedef typename TestCamera::Pose Pose;
typedef typename TestCamera::LineSegment Obstacle;

typedef TestLocalizer::StateIdx StateIdx;

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

// When placing the cameras on the robot, set them all kCameraOffset out from
// the center, to test that we really can handle cameras that aren't at the
// center-of-mass.
constexpr double kCameraOffset = 0.1;

#if defined(SUPPORT_PLOT)
// Plots a line from a vector of Pose's.
void PlotPlotPts(const ::std::vector<Pose> &poses,
                 const ::std::map<::std::string, ::std::string> &kwargs) {
  ::std::vector<double> x;
  ::std::vector<double> y;
  for (const Pose &p : poses) {
    x.push_back(p.abs_pos().x());
    y.push_back(p.abs_pos().y());
  }
  matplotlibcpp::plot(x, y, kwargs);
}
#endif

struct LocalizerTestParams {
  // Control points for the spline to make the robot follow.
  ::std::array<float, 6> control_pts_x;
  ::std::array<float, 6> control_pts_y;
  // The actual state to start the robot at. By setting voltage errors and the
  // such you can introduce persistent disturbances.
  TestLocalizer::State true_start_state;
  // The initial state of the estimator.
  TestLocalizer::State known_start_state;
  // Whether or not to add Gaussian noise to the sensors and cameras.
  bool noisify;
  // Whether or not to add unmodelled disturbances.
  bool disturb;
  // The tolerances for the estimator and for the trajectory following at
  // the end of the spline:
  double estimate_tolerance;
  double goal_tolerance;
};

class ParameterizedLocalizerTest
    : public ::testing::TestWithParam<LocalizerTestParams> {
 public:
  ::aos::testing::TestSharedMemory shm_;

  // Set up three targets in a row, at (-1, 0), (0, 0), and (1, 0).
  // Make the right-most target (1, 0) be facing away from the camera, and give
  // the middle target some skew.
  // Place one camera facing forward, the other facing backward, and set the
  // robot at (0, -5) with the cameras each 0.1m from the center.
  // Place one obstacle in a place where it can block the left-most target (-1,
  // 0).
  ParameterizedLocalizerTest()
      : field_(),
        targets_(field_.targets()),
        modeled_obstacles_(field_.obstacles()),
        true_obstacles_(field_.obstacles()),
        dt_config_(drivetrain::GetDrivetrainConfig()),
        // Pull the noise for the encoders/gyros from the R matrix:
        encoder_noise_(::std::sqrt(
            dt_config_.make_kf_drivetrain_loop().observer().coefficients().R(
                0, 0))),
        gyro_noise_(::std::sqrt(
            dt_config_.make_kf_drivetrain_loop().observer().coefficients().R(
                2, 2))),
        // As per the comments in localizer.h, we set the field of view and
        // noise parameters on the robot_cameras_ so that they see a bit more
        // than the true_cameras_. The robot_cameras_ are what is passed to the
        // localizer and used to generate "expected" targets. The true_cameras_
        // are what we actually use to generate targets to pass to the
        // localizer.
        robot_cameras_{
            {TestCamera({&robot_pose_, {0.0, kCameraOffset, 0.0}, M_PI_2},
                        M_PI_2 * 1.1, robot_noise_parameters_, targets_,
                        modeled_obstacles_),
             TestCamera({&robot_pose_, {kCameraOffset, 0.0, 0.0}, 0.0},
                        M_PI_2 * 1.1, robot_noise_parameters_, targets_,
                        modeled_obstacles_),
             TestCamera({&robot_pose_, {-kCameraOffset, 0.0, 0.0}, M_PI},
                        M_PI_2 * 1.1, robot_noise_parameters_, targets_,
                        modeled_obstacles_),
             TestCamera({&robot_pose_, {0.0, -kCameraOffset, 0.0}, -M_PI_2},
                        M_PI_2 * 1.1, robot_noise_parameters_, targets_,
                        modeled_obstacles_)}},
        true_cameras_{
            {TestCamera({&true_robot_pose_, {0.0, kCameraOffset, 0.0}, M_PI_2},
                        M_PI_2 * 0.9, true_noise_parameters_, targets_,
                        true_obstacles_),
             TestCamera({&true_robot_pose_, {kCameraOffset, 0.0, 0.0}, 0.0},
                        M_PI_2 * 0.9, true_noise_parameters_, targets_,
                        true_obstacles_),
             TestCamera({&true_robot_pose_, {-kCameraOffset, 0.0, 0.0}, M_PI},
                        M_PI_2 * 0.9, true_noise_parameters_, targets_,
                        true_obstacles_),
             TestCamera(
                 {&true_robot_pose_, {-0.0, -kCameraOffset, 0.0}, -M_PI_2},
                 M_PI_2 * 0.9, true_noise_parameters_, targets_,
                 true_obstacles_)}},
        localizer_(dt_config_, &robot_pose_),
        spline_drivetrain_(dt_config_) {
    // We use the default P() for initialization.
    localizer_.ResetInitialState(t0_, GetParam().known_start_state,
                                 localizer_.P());
  }

  void SetUp() {
    // Turn on -v 1
    FLAGS_v = std::max(FLAGS_v, 1);

    flatbuffers::DetachedBuffer goal_buffer;
    {
      flatbuffers::FlatBufferBuilder fbb;

      flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
          fbb.CreateVector<float>(GetParam().control_pts_x.begin(),
                                  GetParam().control_pts_x.size());

      flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
          fbb.CreateVector<float>(GetParam().control_pts_y.begin(),
                                  GetParam().control_pts_y.size());

      frc971::MultiSpline::Builder multispline_builder(fbb);

      multispline_builder.add_spline_count(1);
      multispline_builder.add_spline_x(spline_x_offset);
      multispline_builder.add_spline_y(spline_y_offset);

      flatbuffers::Offset<frc971::MultiSpline> multispline_offset =
          multispline_builder.Finish();

      frc971::control_loops::drivetrain::SplineGoal::Builder spline_builder(
          fbb);

      spline_builder.add_spline_idx(1);
      spline_builder.add_spline(multispline_offset);

      flatbuffers::Offset<frc971::control_loops::drivetrain::SplineGoal>
          spline_offset = spline_builder.Finish();

      frc971::control_loops::drivetrain::Goal::Builder goal_builder(fbb);

      goal_builder.add_spline(spline_offset);
      goal_builder.add_controller_type(
          frc971::control_loops::drivetrain::ControllerType::SPLINE_FOLLOWER);
      goal_builder.add_spline_handle(1);

      fbb.Finish(goal_builder.Finish());

      goal_buffer = fbb.Release();
    }
    aos::FlatbufferDetachedBuffer<frc971::control_loops::drivetrain::Goal> goal(
        std::move(goal_buffer));

    // Let the spline drivetrain compute the spline.
    while (true) {
      // We need to keep sending the goal.  There are conditions when the
      // trajectory lock isn't grabbed the first time, and we want to keep
      // banging on it to keep trying.  Otherwise we deadlock.
      spline_drivetrain_.SetGoal(&goal.message());

      ::std::this_thread::sleep_for(::std::chrono::milliseconds(5));

      flatbuffers::FlatBufferBuilder fbb;

      flatbuffers::Offset<frc971::control_loops::drivetrain::TrajectoryLogging>
          trajectory_logging_offset =
              spline_drivetrain_.MakeTrajectoryLogging(&fbb);

      ::frc971::control_loops::drivetrain::Status::Builder status_builder(fbb);
      status_builder.add_trajectory_logging(trajectory_logging_offset);
      spline_drivetrain_.PopulateStatus(&status_builder);
      fbb.Finish(status_builder.Finish());
      aos::FlatbufferDetachedBuffer<::frc971::control_loops::drivetrain::Status>
          status(fbb.Release());

      if (status.message().trajectory_logging()->planning_state() ==
          ::frc971::control_loops::drivetrain::PlanningState::PLANNED) {
        break;
      }
    }
    spline_drivetrain_.SetGoal(&goal.message());
  }

  void TearDown() {
    printf("Each iteration of the simulation took on average %f seconds.\n",
           avg_time_.count());
#if defined(SUPPORT_PLOT)
    if (FLAGS_plot) {
      matplotlibcpp::figure();
      matplotlibcpp::plot(simulation_t_, simulation_vl_, {{"label", "Vl"}});
      matplotlibcpp::plot(simulation_t_, simulation_vr_, {{"label", "Vr"}});
      matplotlibcpp::legend();

      matplotlibcpp::figure();
      matplotlibcpp::plot(spline_x_, spline_y_, {{"label", "spline"}});
      matplotlibcpp::plot(simulation_x_, simulation_y_, {{"label", "robot"}});
      matplotlibcpp::plot(estimated_x_, estimated_y_,
                          {{"label", "estimation"}});
      for (const Target &target : targets_) {
        PlotPlotPts(target.PlotPoints(), {{"c", "g"}});
      }
      for (const Obstacle &obstacle : true_obstacles_) {
        PlotPlotPts(obstacle.PlotPoints(), {{"c", "k"}});
      }
      // Go through and plot true/expected camera targets for a few select
      // time-steps.
      ::std::vector<const char *> colors{"m", "y", "c"};
      int jj = 0;
      for (size_t ii = 0; ii < simulation_x_.size(); ii += 400) {
        *true_robot_pose_.mutable_pos() << simulation_x_[ii], simulation_y_[ii],
            0.0;
        true_robot_pose_.set_theta(simulation_theta_[ii]);
        for (const TestCamera &camera : true_cameras_) {
          for (const auto &plot_pts : camera.PlotPoints()) {
            PlotPlotPts(plot_pts, {{"c", colors[jj]}});
          }
        }
        for (const TestCamera &camera : robot_cameras_) {
          *robot_pose_.mutable_pos() << estimated_x_[ii], estimated_y_[ii], 0.0;
          robot_pose_.set_theta(estimated_theta_[ii]);
          const auto &all_plot_pts = camera.PlotPoints();
          *robot_pose_.mutable_pos() = true_robot_pose_.rel_pos();
          robot_pose_.set_theta(true_robot_pose_.rel_theta());
          for (const auto &plot_pts : all_plot_pts) {
            PlotPlotPts(plot_pts, {{"c", colors[jj]}, {"ls", "--"}});
          }
        }
        jj = (jj + 1) % colors.size();
      }
      matplotlibcpp::legend();

      matplotlibcpp::figure();
      matplotlibcpp::plot(
          simulation_t_, spline_x_,
          {{"label", "spline x"}, {"c", "g"}, {"ls", ""}, {"marker", "o"}});
      matplotlibcpp::plot(simulation_t_, simulation_x_,
                          {{"label", "simulated x"}, {"c", "g"}});
      matplotlibcpp::plot(simulation_t_, estimated_x_,
                          {{"label", "estimated x"}, {"c", "g"}, {"ls", "--"}});

      matplotlibcpp::plot(
          simulation_t_, spline_y_,
          {{"label", "spline y"}, {"c", "b"}, {"ls", ""}, {"marker", "o"}});
      matplotlibcpp::plot(simulation_t_, simulation_y_,
                          {{"label", "simulated y"}, {"c", "b"}});
      matplotlibcpp::plot(simulation_t_, estimated_y_,
                          {{"label", "estimated y"}, {"c", "b"}, {"ls", "--"}});

      matplotlibcpp::plot(simulation_t_, simulation_theta_,
                          {{"label", "simulated theta"}, {"c", "r"}});
      matplotlibcpp::plot(
          simulation_t_, estimated_theta_,
          {{"label", "estimated theta"}, {"c", "r"}, {"ls", "--"}});
      matplotlibcpp::legend();

      matplotlibcpp::show();
    }
#endif
  }

 protected:
  // Returns a random number with a gaussian distribution with a mean of zero
  // and a standard deviation of std, if noisify = true.
  // If noisify is false, then returns 0.0.
  double Normal(double std) {
    if (GetParam().noisify) {
      return normal_(gen_) * std;
    }
    return 0.0;
  }
  // Adds random noise to the given target view.
  void Noisify(TestCamera::TargetView *view) {
    view->reading.heading += Normal(view->noise.heading);
    view->reading.distance += Normal(view->noise.distance);
    view->reading.height += Normal(view->noise.height);
    view->reading.skew += Normal(view->noise.skew);
  }
  // The differential equation for the dynamics of the system.
  TestLocalizer::State DiffEq(const TestLocalizer::State &X,
                              const TestLocalizer::Input &U) {
    return localizer_.DiffEq(X, U);
  }

  Field field_;
  ::std::array<Target, Field::kNumTargets> targets_;
  // The obstacles that are passed to the camera objects for the localizer.
  ::std::array<Obstacle, Field::kNumObstacles> modeled_obstacles_;
  // The obstacles that are used for actually simulating the cameras.
  ::std::array<Obstacle, Field::kNumObstacles> true_obstacles_;

  DrivetrainConfig<double> dt_config_;

  // Noise information for the actual simulated cameras (true_*) and the
  // parameters that the localizer should use for modelling the cameras. Note
  // how the max_viewable_distance is larger for the localizer, so that if
  // there is any error in the estimation, it still thinks that it can see
  // any targets that might actually be in range.
  TestCamera::NoiseParameters true_noise_parameters_ = {
      .max_viewable_distance = 10.0,
      .heading_noise = 0.02,
      .nominal_distance_noise = 0.06,
      .nominal_skew_noise = 0.1,
      .nominal_height_noise = 0.01};
  TestCamera::NoiseParameters robot_noise_parameters_ = {
      .max_viewable_distance = 11.0,
      .heading_noise = 0.02,
      .nominal_distance_noise = 0.06,
      .nominal_skew_noise = 0.1,
      .nominal_height_noise = 0.01};

  // Standard deviations of the noise for the encoders/gyro.
  double encoder_noise_, gyro_noise_;

  Pose robot_pose_;
  ::std::array<TestCamera, 4> robot_cameras_;
  Pose true_robot_pose_;
  ::std::array<TestCamera, 4> true_cameras_;

  TestLocalizer localizer_;

  ::frc971::control_loops::drivetrain::SplineDrivetrain spline_drivetrain_;

  // All the data we want to end up plotting.
  ::std::vector<double> simulation_t_;

  ::std::vector<double> spline_x_;
  ::std::vector<double> spline_y_;
  ::std::vector<double> estimated_x_;
  ::std::vector<double> estimated_y_;
  ::std::vector<double> estimated_theta_;
  ::std::vector<double> simulation_x_;
  ::std::vector<double> simulation_y_;
  ::std::vector<double> simulation_theta_;

  ::std::vector<double> simulation_vl_;
  ::std::vector<double> simulation_vr_;

  // Simulation start time
  ::aos::monotonic_clock::time_point t0_;

  // Average duration of each iteration; used for debugging and getting a
  // sanity-check on what performance looks like--uses a real system clock.
  ::std::chrono::duration<double> avg_time_;

  ::std::mt19937 gen_{static_cast<uint32_t>(::aos::testing::RandomSeed())};
  ::std::normal_distribution<> normal_;
};

using ::std::chrono::milliseconds;

// Tests that, when we attempt to follow a spline and use the localizer to
// perform the state estimation, we end up roughly where we should and that
// the localizer has a valid position estimate.
TEST_P(ParameterizedLocalizerTest, SplineTest) {
  // state stores the true state of the robot throughout the simulation.
  TestLocalizer::State state = GetParam().true_start_state;

  ::aos::monotonic_clock::time_point t = t0_;

  // The period with which we should take frames from the cameras. Currently,
  // we just trigger all the cameras at once, rather than offsetting them or
  // anything.
  const int camera_period = 5;  // cycles
  // The amount of time to delay the camera images from when they are taken, for
  // each camera.
  const ::std::array<milliseconds, 4> camera_latencies{
      {milliseconds(45), milliseconds(50), milliseconds(55),
       milliseconds(100)}};

  // A queue of camera frames for each camera so that we can add a time delay to
  // the data coming from the cameras.
  ::std::array<
      ::std::queue<::std::tuple<
          ::aos::monotonic_clock::time_point, const TestCamera *,
          ::aos::SizedArray<TestCamera::TargetView, kNumTargetsPerFrame>>>,
      4>
      camera_queues;

  // Start time, for debugging.
  const auto begin = ::std::chrono::steady_clock::now();

  size_t i;
  for (i = 0; !spline_drivetrain_.IsAtEnd(); ++i) {
    // Get the current state estimate into a matrix that works for the
    // trajectory code.
    ::Eigen::Matrix<double, 5, 1> known_state;
    TestLocalizer::State X_hat = localizer_.X_hat();
    known_state << X_hat(StateIdx::kX, 0), X_hat(StateIdx::kY, 0),
        X_hat(StateIdx::kTheta, 0), X_hat(StateIdx::kLeftVelocity, 0),
        X_hat(StateIdx::kRightVelocity, 0);

    spline_drivetrain_.Update(true, known_state);

    ::frc971::control_loops::drivetrain::OutputT output;
    output.left_voltage = 0;
    output.right_voltage = 0;
    spline_drivetrain_.SetOutput(&output);
    TestLocalizer::Input U(output.left_voltage, output.right_voltage);

    const ::Eigen::Matrix<double, 5, 1> goal_state =
        spline_drivetrain_.CurrentGoalState();
    simulation_t_.push_back(
        ::aos::time::DurationInSeconds(t.time_since_epoch()));
    spline_x_.push_back(goal_state(0, 0));
    spline_y_.push_back(goal_state(1, 0));
    simulation_x_.push_back(state(StateIdx::kX, 0));
    simulation_y_.push_back(state(StateIdx::kY, 0));
    simulation_theta_.push_back(state(StateIdx::kTheta, 0));
    estimated_x_.push_back(known_state(0, 0));
    estimated_y_.push_back(known_state(1, 0));
    estimated_theta_.push_back(known_state(StateIdx::kTheta, 0));

    simulation_vl_.push_back(U(0));
    simulation_vr_.push_back(U(1));
    U(0, 0) = ::std::max(::std::min(U(0, 0), 12.0), -12.0);
    U(1, 0) = ::std::max(::std::min(U(1, 0), 12.0), -12.0);

    state = ::frc971::control_loops::RungeKuttaU(
        [this](const ::Eigen::Matrix<double, 10, 1> &X,
               const ::Eigen::Matrix<double, 2, 1> &U) { return DiffEq(X, U); },
        state, U, ::aos::time::DurationInSeconds(dt_config_.dt));

    // Add arbitrary disturbances at some arbitrary interval. The main points of
    // interest here are that we (a) stop adding disturbances at the very end of
    // the trajectory, to allow us to actually converge to the goal, and (b)
    // scale disturbances by the corruent velocity.
    if (GetParam().disturb && i % 75 == 0) {
      // Scale the disturbance so that when we have near-zero velocity we don't
      // have much disturbance.
      double disturbance_scale = ::std::min(
          1.0, ::std::sqrt(::std::pow(state(StateIdx::kLeftVelocity, 0), 2) +
                           ::std::pow(state(StateIdx::kRightVelocity, 0), 2)) /
                   3.0);
      TestLocalizer::State disturbance;
      disturbance << 0.02, 0.02, 0.001, 0.03, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0;
      disturbance *= disturbance_scale;
      state += disturbance;
    }

    t += dt_config_.dt;
    *true_robot_pose_.mutable_pos() << state(StateIdx::kX, 0),
        state(StateIdx::kY, 0), 0.0;
    true_robot_pose_.set_theta(state(StateIdx::kTheta, 0));
    const double left_enc = state(StateIdx::kLeftEncoder, 0);
    const double right_enc = state(StateIdx::kRightEncoder, 0);

    const double gyro = (state(StateIdx::kRightVelocity, 0) -
                         state(StateIdx::kLeftVelocity, 0)) /
                        dt_config_.robot_radius / 2.0;

    localizer_.UpdateEncodersAndGyro(left_enc + Normal(encoder_noise_),
                                     right_enc + Normal(encoder_noise_),
                                     gyro + Normal(gyro_noise_), U, t);

    for (size_t cam_idx = 0; cam_idx < camera_queues.size(); ++cam_idx) {
      auto &camera_queue = camera_queues[cam_idx];
      // Clear out the camera frames that we are ready to pass to the localizer.
      while (!camera_queue.empty() && ::std::get<0>(camera_queue.front()) <
                                          t - camera_latencies[cam_idx]) {
        const auto tuple = camera_queue.front();
        camera_queue.pop();
        ::aos::monotonic_clock::time_point t_obs = ::std::get<0>(tuple);
        const TestCamera *camera = ::std::get<1>(tuple);
        ::aos::SizedArray<TestCamera::TargetView, kNumTargetsPerFrame> views =
            ::std::get<2>(tuple);
        localizer_.UpdateTargets(*camera, views, t_obs);
      }

      // Actually take all the images and store them in the queue.
      if (i % camera_period == 0) {
        for (size_t jj = 0; jj < true_cameras_.size(); ++jj) {
          const auto target_views = true_cameras_[jj].target_views();
          ::aos::SizedArray<TestCamera::TargetView, kNumTargetsPerFrame>
              pass_views;
          for (size_t ii = 0;
               ii < ::std::min(kNumTargetsPerFrame, target_views.size());
               ++ii) {
            TestCamera::TargetView view = target_views[ii];
            Noisify(&view);
            pass_views.push_back(view);
          }
          camera_queue.emplace(t, &robot_cameras_[jj], pass_views);
        }
      }
    }
  }

  const auto end = ::std::chrono::steady_clock::now();
  avg_time_ = (end - begin) / i;
  TestLocalizer::State estimate_err = state - localizer_.X_hat();
  EXPECT_LT(estimate_err.template topRows<7>().norm(),
            GetParam().estimate_tolerance);
  // Check that none of the states that we actually care about (x/y/theta, and
  // wheel positions/speeds) are too far off, individually:
  EXPECT_LT(estimate_err.template topRows<3>().cwiseAbs().maxCoeff(),
            GetParam().estimate_tolerance / 3.0)
      << "Estimate error: " << estimate_err.transpose();
  Eigen::Matrix<double, 5, 1> final_trajectory_state;
  final_trajectory_state << state(StateIdx::kX, 0), state(StateIdx::kY, 0),
      state(StateIdx::kTheta, 0), state(StateIdx::kLeftVelocity, 0),
      state(StateIdx::kRightVelocity, 0);
  const Eigen::Matrix<double, 5, 1> final_trajectory_state_err =
      final_trajectory_state - spline_drivetrain_.CurrentGoalState();
  EXPECT_LT(final_trajectory_state_err.norm(), GetParam().goal_tolerance)
      << "Goal error: " << final_trajectory_state_err.transpose();
}

INSTANTIATE_TEST_CASE_P(
    LocalizerTest, ParameterizedLocalizerTest,
    ::testing::Values(
        // Checks a "perfect" scenario, where we should track perfectly.
        LocalizerTestParams({
            /*control_pts_x=*/{{0.0, 3.0, 3.0, 0.0, 1.0, 1.0}},
            /*control_pts_y=*/{{-5.0, -5.0, 2.0, 2.0, 2.0, 3.0}},
            (TestLocalizer::State() << 0.0, -5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0)
                .finished(),
            (TestLocalizer::State() << 0.0, -5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0)
                .finished(),
            /*noisify=*/false,
            /*disturb=*/false,
            /*estimate_tolerance=*/1e-5,
            /*goal_tolerance=*/2e-2,
        }),
        // Checks "perfect" estimation, but start off the spline and check
        // that we can still follow it.
        LocalizerTestParams({
            /*control_pts_x=*/{{0.0, 3.0, 3.0, 0.0, 1.0, 1.0}},
            /*control_pts_y=*/{{-5.0, -5.0, 2.0, 2.0, 2.0, 3.0}},
            (TestLocalizer::State() << 0.0, -4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0)
                .finished(),
            (TestLocalizer::State() << 0.0, -4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0)
                .finished(),
            /*noisify=*/false,
            /*disturb=*/false,
            /*estimate_tolerance=*/1e-5,
            /*goal_tolerance=*/2e-2,
        }),
        // Repeats perfect scenario, but add sensor noise.
        LocalizerTestParams({
            /*control_pts_x=*/{{0.0, 3.0, 3.0, 0.0, 1.0, 1.0}},
            /*control_pts_y=*/{{-5.0, -5.0, 2.0, 2.0, 2.0, 3.0}},
            (TestLocalizer::State() << 0.0, -5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0)
                .finished(),
            (TestLocalizer::State() << 0.0, -5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0)
                .finished(),
            /*noisify=*/true,
            /*disturb=*/false,
            /*estimate_tolerance=*/0.4,
            /*goal_tolerance=*/0.4,
        }),
        // Repeats perfect scenario, but add initial estimator error.
        LocalizerTestParams({
            /*control_pts_x=*/{{0.0, 3.0, 3.0, 0.0, 1.0, 1.0}},
            /*control_pts_y=*/{{-5.0, -5.0, 2.0, 2.0, 2.0, 3.0}},
            (TestLocalizer::State() << 0.0, -5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0)
                .finished(),
            (TestLocalizer::State() << 0.1, -5.1, -0.01, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0)
                .finished(),
            /*noisify=*/false,
            /*disturb=*/false,
            /*estimate_tolerance=*/1e-4,
            /*goal_tolerance=*/2e-2,
        }),
        // Repeats perfect scenario, but add voltage + angular errors:
        LocalizerTestParams({
            /*control_pts_x=*/{{0.0, 3.0, 3.0, 0.0, 1.0, 1.0}},
            /*control_pts_y=*/{{-5.0, -5.0, 2.0, 2.0, 2.0, 3.0}},
            (TestLocalizer::State() << 0.0, -5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
             0.5, 0.02)
                .finished(),
            (TestLocalizer::State() << 0.1, -5.1, -0.01, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0)
                .finished(),
            /*noisify=*/false,
            /*disturb=*/false,
            /*estimate_tolerance=*/1e-4,
            /*goal_tolerance=*/2e-2,
        }),
        // Add disturbances while we are driving:
        LocalizerTestParams({
            /*control_pts_x=*/{{0.0, 3.0, 3.0, 0.0, 1.0, 1.0}},
            /*control_pts_y=*/{{-5.0, -5.0, 2.0, 2.0, 2.0, 3.0}},
            (TestLocalizer::State() << 0.0, -5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0)
                .finished(),
            (TestLocalizer::State() << 0.0, -5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0)
                .finished(),
            /*noisify=*/false,
            /*disturb=*/true,
            /*estimate_tolerance=*/3e-2,
            /*goal_tolerance=*/0.15,
        }),
        // Add noise and some initial error in addition:
        LocalizerTestParams({
            /*control_pts_x=*/{{0.0, 3.0, 3.0, 0.0, 1.0, 1.0}},
            /*control_pts_y=*/{{-5.0, -5.0, 2.0, 2.0, 2.0, 3.0}},
            (TestLocalizer::State() << 0.0, -5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0)
                .finished(),
            (TestLocalizer::State() << 0.1, -5.1, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0)
                .finished(),
            /*noisify=*/true,
            /*disturb=*/true,
            /*estimate_tolerance=*/0.2,
            /*goal_tolerance=*/0.5,
        }),
        // Try another spline, just in case the one I was using is special for
        // some reason; this path will also go straight up to a target, to
        // better simulate what might happen when trying to score:
        LocalizerTestParams({
            /*control_pts_x=*/{{0.5, 3.5, 4.0, 8.0, 11.0, 10.2}},
            /*control_pts_y=*/{{1.0, 1.0, -3.0, -2.0, -3.5, -3.65}},
            (TestLocalizer::State() << 0.6, 1.01, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0)
                .finished(),
            (TestLocalizer::State() << 0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0)
                .finished(),
            /*noisify=*/true,
            /*disturb=*/false,
            // TODO(james): Improve tests so that we aren't constantly
            // readjusting the tolerances up.
            /*estimate_tolerance=*/0.3,
            /*goal_tolerance=*/0.7,
        })));

}  // namespace testing
}  // namespace control_loops
}  // namespace y2019
