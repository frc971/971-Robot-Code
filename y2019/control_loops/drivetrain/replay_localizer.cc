#include <fcntl.h>

#include "aos/init.h"
#include "aos/logging/implementations.h"
#include "aos/logging/replay.h"
#include "aos/network/team_number.h"
#include "aos/queue.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/localizer.q.h"
#include "frc971/wpilib/imu.q.h"
#include "gflags/gflags.h"
#include "y2019/constants.h"
#include "y2019/control_loops/drivetrain/drivetrain_base.h"
#include "y2019/control_loops/drivetrain/event_loop_localizer.h"
#if defined(SUPPORT_PLOT)
#include "third_party/matplotlib-cpp/matplotlibcpp.h"
#endif

DEFINE_string(logfile, "", "Path to the logfile to replay.");
// TODO(james): Figure out how to reliably source team number from logfile.
DEFINE_int32(team, 971, "Team number to use for logfile replay.");
DEFINE_int32(plot_duration, 30,
             "Duration, in seconds, to plot after the start time.");
DEFINE_int32(start_offset, 0,
             "Time, in seconds, to start replay plot after the first enable.");

namespace y2019 {
namespace control_loops {
namespace drivetrain {
using ::y2019::constants::Field;

typedef TypedLocalizer<
    constants::Values::kNumCameras, Field::kNumTargets, Field::kNumObstacles,
    EventLoopLocalizer::kMaxTargetsPerFrame, double> TestLocalizer;
typedef typename TestLocalizer::Camera TestCamera;
typedef typename TestCamera::Pose Pose;
typedef typename TestCamera::LineSegment Obstacle;

#if defined(SUPPORT_PLOT)
// Plots a line from a vector of Pose's.
void PlotPlotPts(const ::std::vector<Pose> &poses,
                 const ::std::map<::std::string, ::std::string> &kwargs) {
  ::std::vector<double> x;
  ::std::vector<double> y;
  for (const Pose & p : poses) {
    x.push_back(p.abs_pos().x());
    y.push_back(p.abs_pos().y());
  }
  matplotlibcpp::plot(x, y, kwargs);
}
#endif

class LocalizerReplayer {
  // This class is for replaying logfiles from the 2019 robot and seeing how the
  // localizer performs withi any new changes versus how it performed during the
  // real match. This starts running replay on the first enable and then will
  // actually plot a subset of the run based on the --plot_duration and
  // --start_offset flags.
 public:
  typedef ::frc971::control_loops::DrivetrainQueue::Goal DrivetrainGoal;
  typedef ::frc971::control_loops::DrivetrainQueue::Position DrivetrainPosition;
  typedef ::frc971::control_loops::DrivetrainQueue::Status DrivetrainStatus;
  typedef ::frc971::control_loops::DrivetrainQueue::Output DrivetrainOutput;
  typedef ::frc971::IMUValues IMUValues;
  typedef ::aos::JoystickState JoystickState;

  LocalizerReplayer() : localizer_(&event_loop_, GetDrivetrainConfig()) {
    replayer_.AddDirectQueueSender<CameraFrame>(
        "wpilib_interface.stripped.IMU", "camera_frames",
        ".y2019.control_loops.drivetrain.camera_frames");

    const char *drivetrain = "drivetrain.stripped";

    replayer_.AddHandler<DrivetrainPosition>(
        drivetrain, "position", [this](const DrivetrainPosition &msg) {
          if (has_been_enabled_) {
            localizer_.Update(last_last_U_ * battery_voltage_ / 12.0,
                              msg.sent_time, msg.left_encoder,
                              msg.right_encoder, latest_gyro_, 0.0);
            if (start_time_ <= msg.sent_time &&
                start_time_ + plot_duration_ >= msg.sent_time) {
              t_pos_.push_back(
                  ::std::chrono::duration_cast<::std::chrono::duration<double>>(
                      msg.sent_time.time_since_epoch())
                      .count());
              new_x_.push_back(localizer_.x());
              new_y_.push_back(localizer_.y());
              new_theta_.push_back(localizer_.theta());
              new_left_encoder_.push_back(localizer_.left_encoder());
              new_right_encoder_.push_back(localizer_.right_encoder());
              new_left_velocity_.push_back(localizer_.left_velocity());
              new_right_velocity_.push_back(localizer_.right_velocity());

              left_voltage_.push_back(last_last_U_(0, 0));
              right_voltage_.push_back(last_last_U_(1, 0));
            }
          }
        });

    replayer_.AddHandler<DrivetrainGoal>(
        drivetrain, "goal", [this](const DrivetrainGoal &msg) {
          if (has_been_enabled_ && start_time_ <= msg.sent_time &&
              (start_time_ + plot_duration_ >= msg.sent_time)) {
            t_goal_.push_back(
                ::std::chrono::duration_cast<::std::chrono::duration<double>>(
                    msg.sent_time.time_since_epoch())
                    .count());
            drivetrain_mode_.push_back(msg.controller_type);
            throttle_.push_back(msg.throttle);
            wheel_.push_back(msg.wheel);
          }
        });

    replayer_.AddHandler<DrivetrainStatus>(
        drivetrain, "status", [this](const DrivetrainStatus &msg) {
          if (has_been_enabled_ && start_time_ <= msg.sent_time &&
              (start_time_ + plot_duration_ >= msg.sent_time)) {
            t_status_.push_back(
                ::std::chrono::duration_cast<::std::chrono::duration<double>>(
                    msg.sent_time.time_since_epoch())
                    .count());
            original_x_.push_back(msg.x);
            original_y_.push_back(msg.y);
            original_theta_.push_back(msg.theta);
            original_line_theta_.push_back(msg.line_follow_logging.rel_theta);
            original_line_goal_theta_.push_back(
                msg.line_follow_logging.goal_theta);
            original_line_distance_.push_back(
                msg.line_follow_logging.distance_to_target);
          }
        });

    replayer_.AddHandler<DrivetrainOutput>(
        drivetrain, "output", [this](const DrivetrainOutput &msg) {
          last_last_U_ = last_U_;
          last_U_ << msg.left_voltage, msg.right_voltage;
        });

    replayer_.AddHandler<frc971::control_loops::drivetrain::LocalizerControl>(
        drivetrain, "localizer_control",
        [this](const frc971::control_loops::drivetrain::LocalizerControl &msg) {
          AOS_LOG_STRUCT(DEBUG, "localizer_control", msg);
          localizer_.ResetPosition(msg.sent_time, msg.x, msg.y, msg.theta,
                                   msg.theta_uncertainty,
                                   !msg.keep_current_theta);
        });

    replayer_.AddHandler<JoystickState>(
        "wpilib_interface.stripped.DSReader", "joystick_state",
        [this](const JoystickState &msg) {
          if (msg.enabled && !has_been_enabled_) {
            has_been_enabled_ = true;
            start_time_ =
                msg.sent_time + ::std::chrono::seconds(FLAGS_start_offset);
          }
        });

    replayer_.AddHandler<IMUValues>(
        "wpilib_interface.stripped.IMU", "sending",
        [this](const IMUValues &msg) {
          latest_gyro_ = msg.gyro_z;
          if (has_been_enabled_ && start_time_ <= msg.sent_time &&
              (start_time_ + plot_duration_ >= msg.sent_time)) {
            t_imu_.push_back(
                ::std::chrono::duration_cast<::std::chrono::duration<double>>(
                    msg.sent_time.time_since_epoch())
                    .count());
            lat_accel_.push_back(msg.accelerometer_y);
            long_accel_.push_back(msg.accelerometer_x);
            z_accel_.push_back(msg.accelerometer_z);
          }
        });

    replayer_.AddHandler<::aos::RobotState>(
        "wpilib_interface.stripped.SensorReader", "robot_state",
        [this](const ::aos::RobotState &msg) {
          battery_voltage_ = msg.voltage_battery;
        });
  }

  void ProcessFile(const char *filename) {
    int fd;
    if (strcmp(filename, "-") == 0) {
      fd = STDIN_FILENO;
    } else {
      fd = open(filename, O_RDONLY);
    }
    if (fd == -1) {
      AOS_PLOG(FATAL, "couldn't open file '%s' for reading", filename);
    }

    replayer_.OpenFile(fd);
    ::aos::RawQueue *queue = ::aos::logging::GetLoggingQueue();
    while (!replayer_.ProcessMessage()) {
      const ::aos::logging::LogMessage *const msg =
          static_cast<const ::aos::logging::LogMessage *>(
              queue->ReadMessage(::aos::RawQueue::kNonBlock));
      if (msg != NULL) {
        ::aos::logging::internal::PrintMessage(stdout, *msg);

        queue->FreeMessage(msg);
      }
      continue;
    }
    replayer_.CloseCurrentFile();

    AOS_PCHECK(close(fd));

    Plot();
  }

  void Plot() {
#if defined(SUPPORT_PLOT)
    int start_idx = 0;
    for (const float t : t_pos_) {
      if (t < ::std::chrono::duration_cast<::std::chrono::duration<double>>(
                  start_time_.time_since_epoch())
                      .count() +
                  0.0) {
        ++start_idx;
      } else {
        break;
      }
    }
    ::std::vector<float> trunc_orig_x(original_x_.begin() + start_idx,
                                      original_x_.end());
    ::std::vector<float> trunc_orig_y(original_y_.begin() + start_idx,
                                      original_y_.end());
    ::std::vector<float> trunc_new_x(new_x_.begin() + start_idx, new_x_.end());
    ::std::vector<float> trunc_new_y(new_y_.begin() + start_idx, new_y_.end());
    matplotlibcpp::figure();
    matplotlibcpp::plot(trunc_orig_x, trunc_orig_y,
                        {{"label", "original"}, {"marker", "o"}});
    matplotlibcpp::plot(trunc_new_x, trunc_new_y,
                        {{"label", "new"}, {"marker", "o"}});
    matplotlibcpp::xlim(-0.1, 19.0);
    matplotlibcpp::ylim(-5.0, 5.0);
    Field field;
    for (const Target &target : field.targets()) {
      PlotPlotPts(target.PlotPoints(), {{"c", "g"}});
    }
    for (const Obstacle &obstacle : field.obstacles()) {
      PlotPlotPts(obstacle.PlotPoints(), {{"c", "k"}});
    }
    matplotlibcpp::grid(true);
    matplotlibcpp::legend();

    matplotlibcpp::figure();
    matplotlibcpp::plot(t_status_, original_x_,
                        {{"label", "original x"}, {"marker", "o"}});
    matplotlibcpp::plot(t_status_, original_y_,
                        {{"label", "original y"}, {"marker", "o"}});
    matplotlibcpp::plot(t_status_, original_theta_,
                        {{"label", "original theta"}, {"marker", "o"}});
    matplotlibcpp::plot(t_pos_, new_x_, {{"label", "new x"}, {"marker", "o"}});
    matplotlibcpp::plot(t_pos_, new_y_, {{"label", "new y"}, {"marker", "o"}});
    matplotlibcpp::plot(t_pos_, new_theta_,
                        {{"label", "new theta"}, {"marker", "o"}});
    matplotlibcpp::grid(true);
    matplotlibcpp::legend();

    matplotlibcpp::figure();
    matplotlibcpp::plot(t_pos_, left_voltage_,
                        {{"label", "left voltage"}, {"marker", "o"}});
    matplotlibcpp::plot(t_pos_, right_voltage_,
                        {{"label", "right voltage"}, {"marker", "o"}});
    matplotlibcpp::grid(true);
    matplotlibcpp::legend();

    matplotlibcpp::figure();
    matplotlibcpp::plot(t_pos_, new_left_encoder_,
                        {{"label", "left encoder"}, {"marker", "o"}});
    matplotlibcpp::plot(t_pos_, new_right_encoder_,
                        {{"label", "right encoder"}, {"marker", "o"}});
    matplotlibcpp::plot(t_pos_, new_left_velocity_,
                        {{"label", "left velocity"}, {"marker", "o"}});
    matplotlibcpp::plot(t_pos_, new_right_velocity_,
                        {{"label", "right velocity"}, {"marker", "o"}});
    matplotlibcpp::grid(true);
    matplotlibcpp::legend();

    matplotlibcpp::figure();
    matplotlibcpp::plot(t_goal_, drivetrain_mode_,
                        {{"label", "mode"}, {"marker", "o"}});
    matplotlibcpp::plot(t_goal_, throttle_,
                        {{"label", "throttle"}, {"marker", "o"}});
    matplotlibcpp::plot(t_goal_, wheel_, {{"label", "wheel"}, {"marker", "o"}});
    matplotlibcpp::plot(t_status_, original_line_theta_,
                        {{"label", "relative theta"}, {"marker", "o"}});
    matplotlibcpp::plot(t_status_, original_line_goal_theta_,
                        {{"label", "goal theta"}, {"marker", "o"}});
    matplotlibcpp::plot(t_status_, original_line_distance_,
                        {{"label", "dist to target"}, {"marker", "o"}});
    matplotlibcpp::grid(true);
    matplotlibcpp::legend();

    matplotlibcpp::figure();
    ::std::vector<double> filt_lat_accel_ = FilterValues(lat_accel_, 10);
    ::std::vector<double> filt_long_accel_ = FilterValues(long_accel_, 10);
    ::std::vector<double> filt_z_accel_ = FilterValues(z_accel_, 10);
    matplotlibcpp::plot(t_imu_, filt_lat_accel_,
                        {{"label", "lateral accel"}, {"marker", "o"}});
    matplotlibcpp::plot(t_imu_, filt_long_accel_,
                        {{"label", "longitudinal accel"}, {"marker", "o"}});
    matplotlibcpp::plot(t_imu_, filt_z_accel_,
                        {{"label", "z accel"}, {"marker", "o"}});
    matplotlibcpp::grid(true);
    matplotlibcpp::legend();

    matplotlibcpp::show();
#endif
  }

 private:
  // Do a simple moving average with a width of N over the given raw data and
  // return the results. For calculating values with N/2 of either edge, we
  // assume that the space past the edges is filled with zeroes.
  ::std::vector<double> FilterValues(const ::std::vector<double> &raw, int N) {
    ::std::vector<double> filt;
    for (int ii = 0; ii < static_cast<int>(raw.size()); ++ii) {
      double sum = 0;
      for (int jj = ::std::max(0, ii - N / 2);
           jj < ::std::min(ii + N / 2, static_cast<int>(raw.size()) - 1);
           ++jj) {
        sum += raw[jj];
      }
      filt.push_back(sum / static_cast<double>(N));
    }
    return filt;
  }

  // TODO(james): Move to some sort of virtual event loop.
  ::aos::ShmEventLoop event_loop_;

  EventLoopLocalizer localizer_;
  // Whether the robot has been enabled yet.
  bool has_been_enabled_ = false;
  // Cache of last gyro value to forward to the localizer.
  double latest_gyro_ = 0.0;      // rad/sec
  double battery_voltage_ = 12.0; // volts
  ::Eigen::Matrix<double, 2, 1> last_U_{0, 0};
  ::Eigen::Matrix<double, 2, 1> last_last_U_{0, 0};

  ::aos::logging::linux_code::LogReplayer replayer_;

  // Time at which to start the plot.
  ::aos::monotonic_clock::time_point start_time_;
  // Length of time to plot.
  ::std::chrono::seconds plot_duration_{FLAGS_plot_duration};

  // All the values to plot, organized in blocks by vectors of data that come in
  // at the same time (e.g., status messages come in offset from position
  // messages and so while they are all generally the same frequency, we can end
  // receiving slightly different numbers of messages on different queues).

  // TODO(james): Improve plotting support.
  ::std::vector<double> t_status_;
  ::std::vector<double> original_x_;
  ::std::vector<double> original_y_;
  ::std::vector<double> original_theta_;
  ::std::vector<double> original_line_theta_;
  ::std::vector<double> original_line_goal_theta_;
  ::std::vector<double> original_line_distance_;

  ::std::vector<double> t_pos_;
  ::std::vector<double> new_x_;
  ::std::vector<double> new_y_;
  ::std::vector<double> new_left_encoder_;
  ::std::vector<double> new_right_encoder_;
  ::std::vector<double> new_left_velocity_;
  ::std::vector<double> new_right_velocity_;
  ::std::vector<double> new_theta_;
  ::std::vector<double> left_voltage_;
  ::std::vector<double> right_voltage_;

  ::std::vector<double> t_goal_;
  ::std::vector<double> drivetrain_mode_;
  ::std::vector<double> throttle_;
  ::std::vector<double> wheel_;

  ::std::vector<double> t_imu_;
  ::std::vector<double> lat_accel_;
  ::std::vector<double> z_accel_;
  ::std::vector<double> long_accel_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2019

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  if (FLAGS_logfile.empty()) {
    fprintf(stderr, "Need a file to replay!\n");
    return EXIT_FAILURE;
  }

  ::aos::network::OverrideTeamNumber(FLAGS_team);

  ::aos::InitCreate();

  ::y2019::control_loops::drivetrain::LocalizerReplayer replay;
  replay.ProcessFile(FLAGS_logfile.c_str());

  ::aos::Cleanup();
}
