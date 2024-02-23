#include "y2024/control_loops/superstructure/shooter.h"

#include "aos/flatbuffers.h"
#include "aos/flatbuffers/base.h"
#include "frc971/control_loops/aiming/aiming.h"
#include "y2024/control_loops/superstructure/catapult/catapult_plant.h"
#include "y2024/control_loops/superstructure/collision_avoidance.h"

namespace y2024::control_loops::superstructure {

using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;

constexpr double kMinCurrent = 20.0;
constexpr double kMaxVelocity = 1.0;
constexpr double kCatapultActivationThreshold = 0.01;

Shooter::Shooter(aos::EventLoop *event_loop, const Constants *robot_constants)
    : drivetrain_status_fetcher_(
          event_loop->MakeFetcher<frc971::control_loops::drivetrain::Status>(
              "/drivetrain")),
      superstructure_can_position_fetcher_(
          event_loop
              ->MakeFetcher<y2024::control_loops::superstructure::CANPosition>(
                  "/superstructure/rio")),
      robot_constants_(robot_constants),
      catapult_(
          robot_constants->common()->catapult(),
          robot_constants->robot()->catapult_constants()->zeroing_constants()),
      turret_(
          robot_constants_->common()->turret(),
          robot_constants_->robot()->turret_constants()->zeroing_constants()),
      altitude_(
          robot_constants_->common()->altitude(),
          robot_constants_->robot()->altitude_constants()->zeroing_constants()),
      aimer_(event_loop, robot_constants_),
      interpolation_table_(
          y2024::constants::Values::InterpolationTableFromFlatbuffer(
              robot_constants_->common()->shooter_interpolation_table())) {}

flatbuffers::Offset<y2024::control_loops::superstructure::ShooterStatus>
Shooter::Iterate(
    const y2024::control_loops::superstructure::Position *position,
    const y2024::control_loops::superstructure::ShooterGoal *shooter_goal,
    double *catapult_output, double *altitude_output, double *turret_output,
    double *retention_roller_output, double /*battery_voltage*/,
    aos::monotonic_clock::time_point current_timestamp,
    CollisionAvoidance *collision_avoidance, const double intake_pivot_position,
    double *max_intake_pivot_position, double *min_intake_pivot_position,
    flatbuffers::FlatBufferBuilder *fbb) {
  superstructure_can_position_fetcher_.Fetch();
  drivetrain_status_fetcher_.Fetch();
  CHECK(superstructure_can_position_fetcher_.get() != nullptr);

  double current_retention_position =
      superstructure_can_position_fetcher_->retention_roller()->position();

  double torque_current =
      superstructure_can_position_fetcher_->retention_roller()
          ->torque_current();

  double retention_velocity =
      (current_retention_position - last_retention_position_) /
      std::chrono::duration<double>(current_timestamp - last_timestamp_)
          .count();

  // If our current is over the minimum current and our velocity is under our
  // maximum velocity, then set loaded to true. If we are preloaded set it to
  // true as well.
  //
  // TODO(austin): Debounce piece_loaded?
  bool piece_loaded =
      (torque_current > kMinCurrent && retention_velocity < kMaxVelocity) ||
      (shooter_goal != nullptr && shooter_goal->preloaded());

  aos::fbs::FixedStackAllocator<aos::fbs::Builder<
      frc971::control_loops::
          StaticZeroingSingleDOFProfiledSubsystemGoalStatic>::kBufferSize>
      turret_allocator;

  aos::fbs::Builder<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoalStatic>
      turret_goal_builder(&turret_allocator);

  aos::fbs::FixedStackAllocator<aos::fbs::Builder<
      frc971::control_loops::
          StaticZeroingSingleDOFProfiledSubsystemGoalStatic>::kBufferSize>
      altitude_allocator;

  aos::fbs::Builder<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoalStatic>
      altitude_goal_builder(&altitude_allocator);

  const double distance_to_goal = aimer_.DistanceToGoal();

  // Always retain the game piece if we are enabled.
  if (retention_roller_output != nullptr) {
    *retention_roller_output =
        robot_constants_->common()->retention_roller_voltages()->retaining();
  }

  bool aiming = false;

  // We don't have the note so we should be ready to intake it.
  if (shooter_goal == nullptr || !shooter_goal->auto_aim() ||
      (!piece_loaded && state_ == CatapultState::READY)) {
    PopulateStaticZeroingSingleDOFProfiledSubsystemGoal(
        turret_goal_builder.get(),
        robot_constants_->common()->turret_loading_position());

    PopulateStaticZeroingSingleDOFProfiledSubsystemGoal(
        altitude_goal_builder.get(),
        robot_constants_->common()->altitude_loading_position());
  } else {
    // We have a game piece, lets start aiming.
    if (drivetrain_status_fetcher_.get() != nullptr) {
      aiming = true;
      aimer_.Update(drivetrain_status_fetcher_.get(),
                    frc971::control_loops::aiming::ShotMode::kShootOnTheFly,
                    turret_goal_builder.get());
    }
  }

  // We have a game piece and are being asked to aim.
  constants::Values::ShotParams shot_params;
  if (piece_loaded && shooter_goal != nullptr && shooter_goal->auto_aim() &&
      interpolation_table_.GetInRange(distance_to_goal, &shot_params)) {
    PopulateStaticZeroingSingleDOFProfiledSubsystemGoal(
        altitude_goal_builder.get(), shot_params.shot_altitude_angle);
  }

  // The builder will contain either the auto-aim goal, or the loading goal. Use
  // it if we have no goal, or no subsystem goal, or if we are auto-aiming.
  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *turret_goal = (shooter_goal != nullptr && !shooter_goal->auto_aim() &&
                      shooter_goal->has_turret_position())
                         ? shooter_goal->turret_position()
                         : &turret_goal_builder->AsFlatbuffer();
  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *altitude_goal = (shooter_goal != nullptr && !shooter_goal->auto_aim() &&
                        shooter_goal->has_altitude_position())
                           ? shooter_goal->altitude_position()
                           : &altitude_goal_builder->AsFlatbuffer();

  bool subsystems_in_range =
      (std::abs(turret_.estimated_position() - turret_goal->unsafe_goal()) <
           kCatapultActivationThreshold &&
       std::abs(altitude_.estimated_position() - altitude_goal->unsafe_goal()) <
           kCatapultActivationThreshold &&
       altitude_.estimated_position() >
           robot_constants_->common()->min_altitude_shooting_angle());

  const bool disabled = turret_.Correct(turret_goal, position->turret(),
                                        turret_output == nullptr);

  collision_avoidance->UpdateGoal(
      {.intake_pivot_position = intake_pivot_position,
       .turret_position = turret_.estimated_position()},
      turret_goal->unsafe_goal());

  turret_.set_min_position(collision_avoidance->min_turret_goal());
  turret_.set_max_position(collision_avoidance->max_turret_goal());

  *max_intake_pivot_position = collision_avoidance->max_intake_pivot_goal();
  *min_intake_pivot_position = collision_avoidance->min_intake_pivot_goal();

  // Calculate the loops for a cycle.
  const double voltage = turret_.UpdateController(disabled);

  turret_.UpdateObserver(voltage);

  // Write out all the voltages.
  if (turret_output) {
    *turret_output = voltage;
  }

  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      turret_status_offset = turret_.MakeStatus(fbb);

  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      altitude_status_offset = altitude_.Iterate(
          altitude_goal, position->altitude(), altitude_output, fbb);

  flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      catapult_status_offset;
  {
    // The catapult will never use a provided goal.  We'll always fabricate one
    // for it.
    //
    // Correct handles resetting our state when disabled.
    const bool disabled = catapult_.Correct(nullptr, position->catapult(),
                                            shooter_goal == nullptr);

    catapult_.set_enable_profile(true);
    // We want a trajectory which accelerates up over the first portion of the
    // range of motion, holds top speed for a little bit, then decelerates
    // before it swings too far.
    //
    // We can solve for these 3 parameters through the range of motion.  Top
    // speed is goverened by the voltage headroom we want to have for the
    // controller.
    //
    // Accel can be tuned given the distance to accelerate over, and decel can
    // be solved similarly.
    //
    // accel = v^2 / (2 * x)
    catapult_.mutable_profile()->set_maximum_velocity(
        catapult::kFreeSpeed * catapult::kOutputRatio * 4.0 / 12.0);

    if (disabled) {
      state_ = CatapultState::RETRACTING;
    }

    switch (state_) {
      case CatapultState::READY:
      case CatapultState::LOADED: {
        if (piece_loaded) {
          state_ = CatapultState::LOADED;
        } else {
          state_ = CatapultState::READY;
        }

        const bool catapult_close = CatapultClose();

        if (subsystems_in_range && shooter_goal != nullptr &&
            shooter_goal->fire() && catapult_close && piece_loaded) {
          state_ = CatapultState::FIRING;
        } else {
          catapult_.set_controller_index(0);
          catapult_.mutable_profile()->set_maximum_acceleration(100.0);
          catapult_.mutable_profile()->set_maximum_deceleration(50.0);
          catapult_.set_unprofiled_goal(0.0, 0.0);

          if (!catapult_close) {
            state_ = CatapultState::RETRACTING;
          }
          break;
        }
        [[fallthrough]];
      }
      case CatapultState::FIRING:
        *retention_roller_output =
            robot_constants_->common()->retention_roller_voltages()->spitting();
        catapult_.set_controller_index(0);
        catapult_.mutable_profile()->set_maximum_acceleration(400.0);
        catapult_.mutable_profile()->set_maximum_deceleration(600.0);
        catapult_.set_unprofiled_goal(2.0, 0.0);
        if (CatapultClose()) {
          state_ = CatapultState::RETRACTING;
        } else {
          break;
        }
        [[fallthrough]];
      case CatapultState::RETRACTING:
        catapult_.set_controller_index(0);
        catapult_.mutable_profile()->set_maximum_acceleration(100.0);
        catapult_.mutable_profile()->set_maximum_deceleration(50.0);
        catapult_.set_unprofiled_goal(0.0, 0.0);

        if (CatapultClose()) {
          if (piece_loaded) {
            state_ = CatapultState::LOADED;
          } else {
            state_ = CatapultState::READY;
          }
        }
        break;
    }

    const double voltage = catapult_.UpdateController(disabled);
    catapult_.UpdateObserver(voltage);
    if (catapult_output != nullptr) {
      *catapult_output = voltage;
    }
    catapult_status_offset = catapult_.MakeStatus(fbb);
  }

  flatbuffers::Offset<AimerStatus> aimer_offset;
  if (aiming) {
    aimer_offset = aimer_.PopulateStatus(fbb);
  }

  y2024::control_loops::superstructure::ShooterStatus::Builder status_builder(
      *fbb);
  status_builder.add_turret(turret_status_offset);
  status_builder.add_altitude(altitude_status_offset);
  status_builder.add_catapult(catapult_status_offset);
  status_builder.add_catapult_state(state_);
  if (aiming) {
    status_builder.add_aimer(aimer_offset);
  }

  last_retention_position_ = current_retention_position;
  last_timestamp_ = current_timestamp;
  return status_builder.Finish();
}

}  // namespace y2024::control_loops::superstructure
