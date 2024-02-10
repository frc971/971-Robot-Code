#ifndef FRC971_CONTROL_LOOPS_STATIC_ZEROING_SINGLE_DOF_PROFILED_SUBSYSTEM_H_
#define FRC971_CONTROL_LOOPS_STATIC_ZEROING_SINGLE_DOF_PROFILED_SUBSYSTEM_H_

#include "aos/flatbuffer_merge.h"
#include "frc971/control_loops/profiled_subsystem.h"
#include "frc971/control_loops/profiled_subsystem_static.h"
#include "frc971/control_loops/state_feedback_loop_converters.h"

namespace frc971::control_loops {

inline void PopulateStaticZeroingSingleDOFProfiledSubsystemGoal(
    StaticZeroingSingleDOFProfiledSubsystemGoalStatic *goal_table,
    double unsafe_goal = 0.0, float max_velocity = 0.0,
    float max_acceleration = 0.0, double goal_velocity = 0.0,
    bool ignore_profile = false) {
  goal_table->set_unsafe_goal(unsafe_goal);
  goal_table->set_goal_velocity(goal_velocity);
  goal_table->set_ignore_profile(ignore_profile);

  frc971::ProfileParametersStatic *profile_parameters =
      goal_table->add_profile_params();
  profile_parameters->set_max_velocity(max_velocity);
  profile_parameters->set_max_acceleration(max_acceleration);
}

template <typename ZeroingEstimator>
struct StaticZeroingSingleDOFProfiledSubsystemParams {
  // Maximum voltage while the subsystem is zeroing
  double zeroing_voltage;

  // Maximum voltage while the subsystem is running
  double operating_voltage;

  // Maximum velocity (units/s) and acceleration while State::ZEROING
  ProfileParametersT zeroing_profile_params;

  // Maximum velocity (units/s) and acceleration while State::RUNNING if max
  // velocity or acceleration in goal profile_params is 0
  ProfileParametersT default_profile_params;

  // Maximum range of the subsystem in meters
  ::frc971::constants::Range range;

  // Zeroing constants for the estimator
  typename ZeroingEstimator::ZeroingConstants zeroing_constants;

  // Function that makes the integral loop for the subsystem
  std::function<StateFeedbackLoop<3, 1, 1>()> make_integral_loop;

  // Used by make_integral_loop when constructed from a flatbuffer.
  std::shared_ptr<aos::FlatbufferDetachedBuffer<
      StaticZeroingSingleDOFProfiledSubsystemCommonParams>>
      loop_params;

  StaticZeroingSingleDOFProfiledSubsystemParams(
      double zeroing_voltage_in, double operating_voltage_in,
      const ProfileParametersT &zeroing_profile_params_in,
      const ProfileParametersT &default_profile_params_in,
      const ::frc971::constants::Range &range_in,
      const typename ZeroingEstimator::ZeroingConstants &zeroing_constants_in,
      std::function<StateFeedbackLoop<3, 1, 1>()> make_integral_loop_in)
      : zeroing_voltage(zeroing_voltage_in),
        operating_voltage(operating_voltage_in),
        zeroing_profile_params(zeroing_profile_params_in),
        default_profile_params(default_profile_params_in),
        range(range_in),
        zeroing_constants(zeroing_constants_in),
        make_integral_loop(make_integral_loop_in) {}

  // Constructs the parameters from flatbuffer types.
  StaticZeroingSingleDOFProfiledSubsystemParams(
      const StaticZeroingSingleDOFProfiledSubsystemCommonParams *common,
      const ZeroingEstimator::ZeroingConstants::TableType *zeroing)
      : zeroing_voltage(common->zeroing_voltage()),
        operating_voltage(common->operating_voltage()),
        zeroing_profile_params(
            aos::UnpackFlatbuffer(common->zeroing_profile_params())),
        default_profile_params(
            aos::UnpackFlatbuffer(common->default_profile_params())),
        range(frc971::constants::Range::FromFlatbuffer(common->range())),
        zeroing_constants(aos::UnpackFlatbuffer(zeroing)),
        make_integral_loop([this]() {
          return MakeStateFeedbackLoop<3, 1, 1>(
              *CHECK_NOTNULL(loop_params->message().loop()));
        }),
        loop_params(std::make_shared<aos::FlatbufferDetachedBuffer<
                        StaticZeroingSingleDOFProfiledSubsystemCommonParams>>(
            aos::RecursiveCopyFlatBuffer(common))) {}
  StaticZeroingSingleDOFProfiledSubsystemParams() = default;
  StaticZeroingSingleDOFProfiledSubsystemParams(
      const StaticZeroingSingleDOFProfiledSubsystemParams &) = default;
  StaticZeroingSingleDOFProfiledSubsystemParams &operator=(
      const StaticZeroingSingleDOFProfiledSubsystemParams &) = default;
};

// Class for controlling and motion profiling a single degree of freedom
// subsystem with a zeroing strategy of not moving.
template <typename TZeroingEstimator, typename TProfiledJointStatus,
          typename TSubsystemParams = TZeroingEstimator>
class StaticZeroingSingleDOFProfiledSubsystem {
 public:
  // Constructs the subsystem from flatbuffer types (appropriate when using the
  // constants.h for the subsystem; the constants.json should be preferred for
  // new subsystems).
  StaticZeroingSingleDOFProfiledSubsystem(
      const StaticZeroingSingleDOFProfiledSubsystemParams<TSubsystemParams>
          &params);
  // Constructs the subsystem from flatbuffer types (appropriate when using a
  // constants.json for the subsystem).
  StaticZeroingSingleDOFProfiledSubsystem(
      const StaticZeroingSingleDOFProfiledSubsystemCommonParams *common,
      const TZeroingEstimator::ZeroingConstants::TableType *zeroing)
      : StaticZeroingSingleDOFProfiledSubsystem(
            StaticZeroingSingleDOFProfiledSubsystemParams<TSubsystemParams>{
                common, zeroing}) {}

  using ZeroingEstimator = TZeroingEstimator;
  using ProfiledJointStatus = TProfiledJointStatus;

  // Returns the filtered goal of the profiled subsystem (R)
  double goal(int row) const { return profiled_subsystem_.goal(row, 0); }

  // Returns the zeroing voltage of the subsystem
  double zeroing_voltage() { return params_.zeroing_voltage; }

  // Returns the operating voltage of the subsystem
  double operating_voltage() { return params_.operating_voltage; }

  // Sets further constraints on the range constant
  void set_min_position(double min_position) { min_position_ = min_position; }

  void set_max_position(double max_position) { max_position_ = max_position; }

  // Sets a temporary acceleration limit.  No accelerations faster than this
  // may be commanded.
  void set_max_acceleration(double max_acceleration) {
    max_acceleration_ = max_acceleration;
  }
  // Clears the acceleration limit.
  void clear_max_acceleration() {
    max_acceleration_ = std::numeric_limits<float>::infinity();
  }

  // Resets constrained min/max position
  void clear_min_position() { min_position_ = params_.range.lower_hard; }
  void clear_max_position() { max_position_ = params_.range.upper_hard; }

  // Sets the unprofiled goal which UpdateController will go to.
  void set_unprofiled_goal(double position, double velocity);
  // Changes the profile parameters for UpdateController to track.
  void AdjustProfile(double velocity, double acceleration);

  // Returns the current position
  double position() const { return profiled_subsystem_.position(); }

  // Returns the most recently corrected state.
  Eigen::Vector3d estimated_state() const {
    return profiled_subsystem_.X_hat();
  }
  double estimated_position() const { return estimated_state()(0, 0); }
  double estimated_velocity() const { return estimated_state()(1, 0); }

  // Corrects the internal state, adjusts limits, and sets nominal goals.
  // Returns true if the controller should run.
  bool Correct(const StaticZeroingSingleDOFProfiledSubsystemGoal *goal,
               const typename ZeroingEstimator::Position *position,
               bool disabled);

  // Computes the feedback and feed forward steps for the current iteration.
  // disabled should be true if the controller is disabled from Correct or
  // another source.
  double UpdateController(bool disabled);

  // Predicts the observer state with the applied voltage.
  void UpdateObserver(double voltage);

  // Returns the current status.
  flatbuffers::Offset<ProfiledJointStatus> MakeStatus(
      flatbuffers::FlatBufferBuilder *status_fbb);

  // Iterates the controller with the provided goal.
  flatbuffers::Offset<ProfiledJointStatus> Iterate(
      const StaticZeroingSingleDOFProfiledSubsystemGoal *goal,
      const typename ZeroingEstimator::Position *position, double *output,
      flatbuffers::FlatBufferBuilder *status_fbb);

  // Sets the current profile state to solve from.  Useful for when an
  // external controller gives back control and we want the trajectory
  // generator to take over control again.
  void ForceGoal(double goal, double goal_velocity);

  // Resets the profiled subsystem and returns to uninitialized
  void Reset();

  void TriggerEstimatorError() { profiled_subsystem_.TriggerEstimatorError(); }

  void Estop() { state_ = State::ESTOP; }

  void set_controller_index(int index) {
    profiled_subsystem_.set_controller_index(index);
  }

  enum class State : int32_t {
    UNINITIALIZED,
    DISABLED_INITIALIZED,
    ZEROING,
    RUNNING,
    ESTOP,
  };

  bool zeroed() const { return profiled_subsystem_.zeroed(); }
  bool estopped() const { return state() == State::ESTOP; }

  State state() const { return state_; }

  bool running() const { return state_ == State::RUNNING; }

  // Returns the controller.
  const StateFeedbackLoop<3, 1, 1> &controller() const {
    return profiled_subsystem_.controller();
  }

 private:
  State state_ = State::UNINITIALIZED;
  double min_position_, max_position_;
  float max_acceleration_ = std::numeric_limits<float>::infinity();

  const StaticZeroingSingleDOFProfiledSubsystemParams<TSubsystemParams> params_;

  ::frc971::control_loops::SingleDOFProfiledSubsystem<ZeroingEstimator>
      profiled_subsystem_;
};

template <typename ZeroingEstimator, typename ProfiledJointStatus,
          typename SubsystemParams>
StaticZeroingSingleDOFProfiledSubsystem<ZeroingEstimator, ProfiledJointStatus,
                                        SubsystemParams>::
    StaticZeroingSingleDOFProfiledSubsystem(
        const StaticZeroingSingleDOFProfiledSubsystemParams<SubsystemParams>
            &params)
    : params_(params),
      profiled_subsystem_(
          ::std::unique_ptr<
              ::frc971::control_loops::SimpleCappedStateFeedbackLoop<3, 1, 1>>(
              new ::frc971::control_loops::SimpleCappedStateFeedbackLoop<
                  3, 1, 1>(params_.make_integral_loop())),
          params.zeroing_constants, params.range,
          params.default_profile_params.max_velocity,
          params.default_profile_params.max_acceleration) {
  Reset();
};

template <typename ZeroingEstimator, typename ProfiledJointStatus,
          typename SubsystemParams>
void StaticZeroingSingleDOFProfiledSubsystem<
    ZeroingEstimator, ProfiledJointStatus, SubsystemParams>::Reset() {
  state_ = State::UNINITIALIZED;
  clear_min_position();
  clear_max_position();
  profiled_subsystem_.Reset();
}

template <typename ZeroingEstimator, typename ProfiledJointStatus,
          typename SubsystemParams>
bool StaticZeroingSingleDOFProfiledSubsystem<
    ZeroingEstimator, ProfiledJointStatus, SubsystemParams>::
    Correct(const StaticZeroingSingleDOFProfiledSubsystemGoal *goal,
            const typename ZeroingEstimator::Position *position,
            bool disabled) {
  CHECK_NOTNULL(position);
  profiled_subsystem_.Correct(*position);

  if (profiled_subsystem_.error()) {
    state_ = State::ESTOP;
  }

  switch (state_) {
    case State::UNINITIALIZED:
      if (profiled_subsystem_.initialized()) {
        state_ = State::DISABLED_INITIALIZED;
      }
      disabled = true;
      break;
    case State::DISABLED_INITIALIZED:
      // Wait here until we are either fully zeroed while disabled, or we become
      // enabled.
      if (disabled) {
        if (profiled_subsystem_.zeroed()) {
          state_ = State::RUNNING;
        }
      } else {
        state_ = State::ZEROING;
      }

      // Set the goals to where we are now so when we start back up, we don't
      // jump.
      profiled_subsystem_.ForceGoal(profiled_subsystem_.position());
      // Set up the profile to be the zeroing profile.
      profiled_subsystem_.AdjustProfile(
          params_.zeroing_profile_params.max_velocity,
          params_.zeroing_profile_params.max_acceleration);

      // We are not ready to start doing anything yet.
      disabled = true;
      break;
    case State::ZEROING:
      // Now, zero by actively holding still.
      if (profiled_subsystem_.zeroed()) {
        state_ = State::RUNNING;
      } else if (disabled) {
        state_ = State::DISABLED_INITIALIZED;
      }
      break;

    case State::RUNNING: {
      if (disabled) {
        // Reset the profile to the current position so it starts from here when
        // we get re-enabled.
        profiled_subsystem_.ForceGoal(profiled_subsystem_.position());
      }

      if (goal) {
        if (goal->profile_params()) {
          AdjustProfile(goal->profile_params()->max_velocity(),
                        goal->profile_params()->max_acceleration());
        } else {
          AdjustProfile(profiled_subsystem_.default_velocity(),
                        profiled_subsystem_.default_acceleration());
        }

        if (goal->has_ignore_profile()) {
          profiled_subsystem_.set_enable_profile(!goal->ignore_profile());
        }
        set_unprofiled_goal(goal->unsafe_goal(), goal->goal_velocity());
      }
    } break;

    case State::ESTOP:
      AOS_LOG(ERROR, "Estop\n");
      disabled = true;
      break;
  }

  // Set the voltage limits.
  const double max_voltage = (state_ == State::RUNNING)
                                 ? params_.operating_voltage
                                 : params_.zeroing_voltage;

  profiled_subsystem_.set_max_voltage({{max_voltage}});

  return disabled;
}

template <typename ZeroingEstimator, typename ProfiledJointStatus,
          typename SubsystemParams>
void StaticZeroingSingleDOFProfiledSubsystem<
    ZeroingEstimator, ProfiledJointStatus,
    SubsystemParams>::set_unprofiled_goal(double goal, double goal_velocity) {
  if (goal < min_position_) {
    VLOG(1) << "Limiting to " << min_position_ << " from " << goal;
    goal = min_position_;
  }
  if (goal > max_position_) {
    VLOG(1) << "Limiting to " << max_position_ << " from " << goal;
    goal = max_position_;
  }
  profiled_subsystem_.set_unprofiled_goal(goal, goal_velocity);
}

template <typename ZeroingEstimator, typename ProfiledJointStatus,
          typename SubsystemParams>
void StaticZeroingSingleDOFProfiledSubsystem<
    ZeroingEstimator, ProfiledJointStatus,
    SubsystemParams>::AdjustProfile(double velocity, double acceleration) {
  profiled_subsystem_.AdjustProfile(
      velocity, std::min(acceleration, static_cast<double>(max_acceleration_)));
}

template <typename ZeroingEstimator, typename ProfiledJointStatus,
          typename SubsystemParams>
flatbuffers::Offset<ProfiledJointStatus>
StaticZeroingSingleDOFProfiledSubsystem<ZeroingEstimator, ProfiledJointStatus,
                                        SubsystemParams>::
    Iterate(const StaticZeroingSingleDOFProfiledSubsystemGoal *goal,
            const typename ZeroingEstimator::Position *position, double *output,
            flatbuffers::FlatBufferBuilder *status_fbb) {
  const bool disabled = Correct(goal, position, output == nullptr);

  // Calculate the loops for a cycle.
  const double voltage = UpdateController(disabled);

  UpdateObserver(voltage);

  // Write out all the voltages.
  if (output) {
    *output = voltage;
  }

  return MakeStatus(status_fbb);
}

template <typename ZeroingEstimator, typename ProfiledJointStatus,
          typename SubsystemParams>
double StaticZeroingSingleDOFProfiledSubsystem<
    ZeroingEstimator, ProfiledJointStatus,
    SubsystemParams>::UpdateController(bool disabled) {
  return profiled_subsystem_.UpdateController(disabled);
}

template <typename ZeroingEstimator, typename ProfiledJointStatus,
          typename SubsystemParams>
void StaticZeroingSingleDOFProfiledSubsystem<
    ZeroingEstimator, ProfiledJointStatus,
    SubsystemParams>::UpdateObserver(double voltage) {
  profiled_subsystem_.UpdateObserver(voltage);
}

template <typename ZeroingEstimator, typename ProfiledJointStatus,
          typename SubsystemParams>
void StaticZeroingSingleDOFProfiledSubsystem<
    ZeroingEstimator, ProfiledJointStatus,
    SubsystemParams>::ForceGoal(double goal, double goal_velocity) {
  profiled_subsystem_.ForceGoal(goal, goal_velocity);
}

template <typename ZeroingEstimator, typename ProfiledJointStatus,
          typename SubsystemParams>
flatbuffers::Offset<ProfiledJointStatus>
StaticZeroingSingleDOFProfiledSubsystem<
    ZeroingEstimator, ProfiledJointStatus,
    SubsystemParams>::MakeStatus(flatbuffers::FlatBufferBuilder *status_fbb) {
  CHECK_NOTNULL(status_fbb);

  typename ProfiledJointStatus::Builder status_builder =
      profiled_subsystem_
          .template BuildStatus<typename ProfiledJointStatus::Builder>(
              status_fbb);

  status_builder.add_estopped(estopped());
  status_builder.add_state(static_cast<int32_t>(state_));
  return status_builder.Finish();
}

}  // namespace frc971::control_loops

#endif  // FRC971_CONTROL_LOOPS_STATIC_ZEROING_SINGLE_DOF_PROFILED_SUBSYSTEM_H_
