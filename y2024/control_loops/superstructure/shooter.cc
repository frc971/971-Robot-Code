#include "y2024/control_loops/superstructure/shooter.h"

#include "aos/flatbuffers.h"
#include "aos/flatbuffers/base.h"
#include "frc971/control_loops/aiming/aiming.h"
#include "y2024/control_loops/superstructure/catapult/catapult_plant.h"
#include "y2024/control_loops/superstructure/collision_avoidance.h"

namespace y2024::control_loops::superstructure {

using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;

constexpr double kCatapultActivationTurretThreshold = 0.03;
constexpr double kCatapultActivationAltitudeThreshold = 0.01;

Shooter::Shooter(aos::EventLoop *event_loop, const Constants *robot_constants)
    : drivetrain_status_fetcher_(
          event_loop->MakeFetcher<frc971::control_loops::drivetrain::Status>(
              "/drivetrain")),
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
              robot_constants_->common()->shooter_interpolation_table())),
      interpolation_table_shuttle_(
          y2024::constants::Values::InterpolationTableFromFlatbuffer(
              robot_constants_->common()
                  ->shooter_shuttle_interpolation_table())),
      debouncer_(std::chrono::milliseconds(100), std::chrono::milliseconds(8)) {
}

flatbuffers::Offset<y2024::control_loops::superstructure::ShooterStatus>
Shooter::Iterate(
    const y2024::control_loops::superstructure::Position *position,
    const y2024::control_loops::superstructure::ShooterGoal *shooter_goal,
    bool fire, double *catapult_output, double *altitude_output,
    double *turret_output, double *retention_roller_output,
    double *retention_roller_stator_current_limit, double /*battery_voltage*/,
    CollisionAvoidance *collision_avoidance, const double extend_position,
    const double extend_goal, double *max_extend_position,
    double *min_extend_position, const double intake_pivot_position,
    double *max_intake_pivot_position, double *min_intake_pivot_position,
    NoteGoal requested_note_goal, flatbuffers::FlatBufferBuilder *fbb,
    aos::monotonic_clock::time_point monotonic_now, bool climbing) {
  drivetrain_status_fetcher_.Fetch();

  // If our current is over the minimum current and our velocity is under our
  // maximum velocity, then set loaded to true. If we are preloaded set it to
  // true as well.
  debouncer_.Update(position->catapult_beambreak() ||
                        (shooter_goal != nullptr && shooter_goal->preloaded()),
                    monotonic_now);
  const bool piece_loaded = debouncer_.state();

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
      auto_aim_allocator;

  aos::fbs::Builder<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoalStatic>
      auto_aim_goal_builder(&auto_aim_allocator);

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
        robot_constants_->common()->retention_roller_voltages()->intaking();

    if (piece_loaded) {
      *retention_roller_output =
          robot_constants_->common()->retention_roller_voltages()->retaining();
      *retention_roller_stator_current_limit =
          robot_constants_->common()
              ->current_limits()
              ->slower_retention_roller_stator_current_limit();
    } else {
      *retention_roller_stator_current_limit =
          robot_constants_->common()
              ->current_limits()
              ->retention_roller_stator_current_limit();
    }
  }

  bool aiming = false;

  if (requested_note_goal == NoteGoal::AMP ||
      requested_note_goal == NoteGoal::TRAP) {
    // Being asked to amp, lift the altitude up.
    PopulateStaticZeroingSingleDOFProfiledSubsystemGoal(
        turret_goal_builder.get(),
        robot_constants_->common()->turret_loading_position());

    PopulateStaticZeroingSingleDOFProfiledSubsystemGoal(
        altitude_goal_builder.get(),
        robot_constants_->common()->altitude_avoid_extend_collision_position());
  } else if (shooter_goal == nullptr ||
             (shooter_goal->auto_aim() == AutoAimMode::NONE) ||
             (!piece_loaded && state_ == CatapultState::READY)) {
    // We don't have the note so we should be ready to intake it.
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
    }
  }

  // Auto aim builder is a dummy so we get a status when we aren't aiming.
  aimer_.Update(
      drivetrain_status_fetcher_.get(),
      frc971::control_loops::aiming::ShotMode::kShootOnTheFly,
      aiming ? turret_goal_builder.get() : auto_aim_goal_builder.get(),
      shooter_goal != nullptr ? shooter_goal->auto_aim() : AutoAimMode::NONE);

  // We have a game piece and are being asked to aim.
  constants::Values::ShotParams shot_params;
  frc971::shooter_interpolation::InterpolationTable<
      y2024::constants::Values::ShotParams> *interpolation_table =
      (shooter_goal != nullptr &&
       shooter_goal->auto_aim() != AutoAimMode::SPEAKER)
          ? &interpolation_table_shuttle_
          : &interpolation_table_;
  if ((piece_loaded || state_ == CatapultState::FIRING) &&
      shooter_goal != nullptr &&
      (shooter_goal->auto_aim() != AutoAimMode::NONE) &&
      interpolation_table->GetInRange(distance_to_goal, &shot_params)) {
    PopulateStaticZeroingSingleDOFProfiledSubsystemGoal(
        altitude_goal_builder.get(), shot_params.shot_altitude_angle);
  }

  // The builder will contain either the auto-aim goal, or the loading goal. Use
  // it if we have no goal, or no subsystem goal, or if we are auto-aiming.

  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *turret_goal =
          (shooter_goal != nullptr &&
           (shooter_goal->auto_aim() == AutoAimMode::NONE) &&
           (piece_loaded || state_ == CatapultState::FIRING || climbing) &&
           shooter_goal->has_turret_position())
              ? shooter_goal->turret_position()
              : &turret_goal_builder->AsFlatbuffer();

  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *altitude_goal = (shooter_goal != nullptr &&
                        (shooter_goal->auto_aim() == AutoAimMode::NONE) &&
                        (piece_loaded || state_ == CatapultState::FIRING) &&
                        shooter_goal->has_altitude_position())
                           ? shooter_goal->altitude_position()
                           : &altitude_goal_builder->AsFlatbuffer();

  // TODO(austin): goal limit...
  const bool turret_in_range =
      (std::abs(turret_.estimated_position() - turret_goal->unsafe_goal()) <
       kCatapultActivationTurretThreshold) &&
      (std::abs(turret_goal->goal_velocity()) < 0.2) &&
      (std::abs(turret_.estimated_velocity()) < 1.0);
  const bool altitude_in_range =
      (std::abs(altitude_.estimated_position() - altitude_goal->unsafe_goal()) <
       (altitude_goal->unsafe_goal() > 0.75
            ? (5 * kCatapultActivationAltitudeThreshold)
            : kCatapultActivationAltitudeThreshold)) &&
      (std::abs(altitude_.estimated_velocity()) < 0.4);
  const bool altitude_above_min_angle =
      (altitude_.estimated_position() >
       robot_constants_->common()->min_altitude_shooting_angle());

  bool subsystems_in_range =
      (turret_in_range && altitude_in_range && altitude_above_min_angle);

  const bool disabled = turret_.Correct(turret_goal, position->turret(),
                                        turret_output == nullptr);

  // Zero out extend goal and position if "disable_extend" is true
  collision_avoidance->UpdateGoal(
      {.intake_pivot_position = intake_pivot_position,
       .turret_position = turret_.estimated_position(),
       .extend_position =
           ((!robot_constants_->robot()->disable_extend()) ? extend_position
                                                           : 0.0)},
      turret_goal->unsafe_goal(),
      ((!robot_constants_->robot()->disable_extend()) ? extend_goal : 0.0));

  if (!CatapultRetracted()) {
    altitude_.set_min_position(
        robot_constants_->common()->min_altitude_shooting_angle());
  } else {
    altitude_.clear_min_position();
  }

  turret_.set_min_position(collision_avoidance->min_turret_goal());
  turret_.set_max_position(collision_avoidance->max_turret_goal());

  *max_intake_pivot_position = collision_avoidance->max_intake_pivot_goal();
  *min_intake_pivot_position = collision_avoidance->min_intake_pivot_goal();

  *max_extend_position = collision_avoidance->max_extend_goal();
  *min_extend_position = collision_avoidance->min_extend_goal();

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
                                            catapult_output == nullptr);

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
        catapult::kFreeSpeed * catapult::kOutputRatio * 5.5 / 12.0);

    if (disabled) {
      state_ = CatapultState::RETRACTING;
    }

    constexpr double kLoadingAcceleration = 40.0;
    constexpr double kLoadingDeceleration = 20.0;

    switch (state_) {
      case CatapultState::READY:
        [[fallthrough]];
      case CatapultState::LOADED: {
        aimer_.latch_note_current(false);
        if (piece_loaded) {
          state_ = CatapultState::LOADED;
        } else {
          aimer_.IndicateReady();
          state_ = CatapultState::READY;
        }

        const bool catapult_close = CatapultClose();

        if (subsystems_in_range && shooter_goal != nullptr && fire &&
            catapult_close && piece_loaded) {
          state_ = CatapultState::FIRING;
          max_catapult_goal_velocity_ = catapult_.goal(1);
        } else {
          catapult_.set_controller_index(0);
          catapult_.mutable_profile()->set_maximum_acceleration(
              kLoadingAcceleration);
          catapult_.mutable_profile()->set_maximum_deceleration(
              kLoadingDeceleration);
          catapult_.set_unprofiled_goal(0.0, 0.0);

          if (!catapult_close) {
            state_ = CatapultState::RETRACTING;
          }
          break;
        }
        [[fallthrough]];
      }
      case CatapultState::FIRING:
        aimer_.latch_note_current(true);
        *retention_roller_output =
            robot_constants_->common()->retention_roller_voltages()->spitting();
        *retention_roller_stator_current_limit =
            robot_constants_->common()
                ->current_limits()
                ->shooting_retention_roller_stator_current_limit();
        max_catapult_goal_velocity_ =
            std::max(max_catapult_goal_velocity_, catapult_.goal(1));

        if (max_catapult_goal_velocity_ > catapult_.goal(1) + 0.1) {
          catapult_.set_controller_index(2);
        } else {
          catapult_.set_controller_index(1);
        }
        catapult_.mutable_profile()->set_maximum_acceleration(400.0);
        catapult_.mutable_profile()->set_maximum_deceleration(1000.0);
        catapult_.set_unprofiled_goal(2.45, 0.0);
        if (CatapultClose()) {
          state_ = CatapultState::RETRACTING;
          ++shot_count_;
        } else {
          break;
        }
        [[fallthrough]];
      case CatapultState::RETRACTING:
        aimer_.latch_note_current(false);
        catapult_.set_controller_index(0);
        catapult_.mutable_profile()->set_maximum_acceleration(
            kLoadingAcceleration);
        catapult_.mutable_profile()->set_maximum_deceleration(
            kLoadingDeceleration);
        // TODO: catapult_return_position
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
  aimer_offset = aimer_.PopulateStatus(fbb);

  y2024::control_loops::superstructure::ShooterStatus::Builder status_builder(
      *fbb);
  status_builder.add_turret(turret_status_offset);
  status_builder.add_altitude(altitude_status_offset);
  status_builder.add_catapult(catapult_status_offset);
  status_builder.add_catapult_state(state_);
  status_builder.add_turret_in_range(turret_in_range);
  status_builder.add_altitude_in_range(altitude_in_range);
  status_builder.add_altitude_above_min_angle(altitude_above_min_angle);
  status_builder.add_auto_aiming(aiming);
  status_builder.add_aimer(aimer_offset);

  return status_builder.Finish();
}

}  // namespace y2024::control_loops::superstructure
