#include "y2022/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"
#include "y2022/control_loops/superstructure/collision_avoidance.h"

namespace y2022 {
namespace control_loops {
namespace superstructure {

using frc971::control_loops::AbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::RelativeEncoderProfiledJointStatus;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               std::shared_ptr<const constants::Values> values,
                               const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                    name),
      values_(values),
      climber_(values_->climber.subsystem_params),
      intake_front_(values_->intake_front.subsystem_params),
      intake_back_(values_->intake_back.subsystem_params),
      turret_(values_->turret.subsystem_params),
      catapult_(*values_),
      drivetrain_status_fetcher_(
          event_loop->MakeFetcher<frc971::control_loops::drivetrain::Status>(
              "/drivetrain")),
      can_position_fetcher_(
          event_loop->MakeFetcher<CANPosition>("/superstructure")) {
  event_loop->SetRuntimeRealtimePriority(30);
}

void Superstructure::RunIteration(const Goal *unsafe_goal,
                                  const Position *position,
                                  aos::Sender<Output>::Builder *output,
                                  aos::Sender<Status>::Builder *status) {
  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
    intake_front_.Reset();
    intake_back_.Reset();
    turret_.Reset();
    climber_.Reset();
    catapult_.Reset();
  }

  OutputT output_struct;

  aos::FlatbufferFixedAllocatorArray<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal, 64>
      turret_goal_buffer;

  const aos::monotonic_clock::time_point timestamp =
      event_loop()->context().monotonic_event_time;

  drivetrain_status_fetcher_.Fetch();
  const float velocity = robot_velocity();

  const turret::Aimer::Goal *auto_aim_goal = nullptr;
  if (drivetrain_status_fetcher_.get() != nullptr) {
    aimer_.Update(drivetrain_status_fetcher_.get(),
                  turret::Aimer::ShotMode::kShootOnTheFly);
    auto_aim_goal = aimer_.TurretGoal();
  }

  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *turret_goal = nullptr;
  double roller_speed_compensated_front = 0.0;
  double roller_speed_compensated_back = 0.0;
  double transfer_roller_speed = 0.0;
  double flipper_arms_voltage = 0.0;

  if (unsafe_goal != nullptr) {
    roller_speed_compensated_front =
        unsafe_goal->roller_speed_front() +
        std::max(velocity * unsafe_goal->roller_speed_compensation(), 0.0);

    roller_speed_compensated_back =
        unsafe_goal->roller_speed_back() -
        std::min(velocity * unsafe_goal->roller_speed_compensation(), 0.0);

    transfer_roller_speed = unsafe_goal->transfer_roller_speed();

    turret_goal =
        unsafe_goal->auto_aim() ? auto_aim_goal : unsafe_goal->turret();
  }

  // Supersturcture state machine:
  // 1. IDLE: Wait until an intake beambreak is triggerred, meaning that a ball
  // is being intaked. This means that the transfer rollers have a ball. If
  // we've been waiting here for too long without any beambreak triggered, the
  // ball got lost, so reset.
  // 2. TRANSFERRING: Until the turret reaches the loading position where the
  // ball can be transferred into the catapult, wiggle the ball in place.
  // Once the turret reaches the loading position, send the ball forward with
  // the transfer rollers until the turret beambreak is triggered.
  // If we have been in this state for too long, the ball probably got lost so
  // reset back to IDLE.
  // 3. LOADING: To load the ball into the catapult, put the flippers at the
  // feeding speed. Wait for a timeout, and then wait until the ball has gone
  // past the turret beambreak and the flippers have stopped moving, meaning
  // that the ball is fully loaded in the catapult.
  // 4. LOADED: Wait until the user asks us to fire to transition to the
  // shooting stage. If asked to cancel the shot, reset back to the IDLE state.
  // 5. SHOOTING: Open the flippers to get ready for the shot. If they don't
  // open quickly enough, try reseating the ball and going back to the LOADING
  // stage, which moves the flippers in the opposite direction first.
  // Now, hold the flippers open and wait until the turret has reached its
  // aiming goal. Once the turret is ready, tell the catapult to fire.
  // If the flippers move back for some reason now, it could damage the
  // catapult, so estop it. Otherwise, wait until the catapult shoots a ball and
  // goes back to its return position. We have now finished the shot, so return
  // to IDLE.

  const bool is_spitting = ((intake_state_ == IntakeState::INTAKE_FRONT_BALL &&
                             transfer_roller_speed < 0) ||
                            (intake_state_ == IntakeState::INTAKE_BACK_BALL &&
                             transfer_roller_speed > 0));

  // Intake handling should happen regardless of the turret state
  if (position->intake_beambreak_front() || position->intake_beambreak_back()) {
    if (intake_state_ == IntakeState::NO_BALL) {
      if (position->intake_beambreak_front()) {
        intake_state_ = IntakeState::INTAKE_FRONT_BALL;
      } else if (position->intake_beambreak_back()) {
        intake_state_ = IntakeState::INTAKE_BACK_BALL;
      }
    }

    intake_beambreak_timer_ = timestamp;
  }

  if (timestamp >
      intake_beambreak_timer_ + constants::Values::kBallLostTime()) {
    intake_state_ = IntakeState::NO_BALL;
  }

  if (intake_state_ != IntakeState::NO_BALL) {
    // Block intaking in
    roller_speed_compensated_front = 0.0;
    roller_speed_compensated_back = 0.0;

    const double wiggle_voltage =
        (intake_state_ == IntakeState::INTAKE_FRONT_BALL
             ? constants::Values::kTransferRollerFrontWiggleVoltage()
             : constants::Values::kTransferRollerBackWiggleVoltage());
    // Wiggle transfer rollers: send the ball back and forth while waiting
    // for the turret or waiting for another shot to be completed
    if ((intake_state_ == IntakeState::INTAKE_FRONT_BALL &&
         position->intake_beambreak_front()) ||
        (intake_state_ == IntakeState::INTAKE_BACK_BALL &&
         position->intake_beambreak_back())) {
      transfer_roller_speed = -wiggle_voltage;
    } else {
      transfer_roller_speed = wiggle_voltage;
    }
  }

  switch (state_) {
    case SuperstructureState::IDLE: {
      if (is_spitting) {
        intake_state_ = IntakeState::NO_BALL;
      }

      if (intake_state_ == IntakeState::NO_BALL ||
          !(position->intake_beambreak_front() ||
            position->intake_beambreak_back())) {
        break;
      }

      state_ = SuperstructureState::TRANSFERRING;
      // Save the side the ball is on for later

      break;
    }
    case SuperstructureState::TRANSFERRING: {
      // If we've been transferring for too long, the ball probably got lost
      if (intake_state_ == IntakeState::NO_BALL) {
        state_ = SuperstructureState::IDLE;
        break;
      }

      double turret_loading_position =
          (intake_state_ == IntakeState::INTAKE_FRONT_BALL
               ? constants::Values::kTurretFrontIntakePos()
               : constants::Values::kTurretBackIntakePos());

      // Turn to the loading position as close to the current position as
      // possible
      // Strategy is copied from frc971/control_loops/aiming/aiming.cc
      turret_loading_position =
          turret_.estimated_position() +
          aos::math::NormalizeAngle(turret_loading_position -
                                    turret_.estimated_position());
      // if out of range, reset back to within +/- pi of zero.
      if (turret_loading_position > constants::Values::kTurretRange().upper ||
          turret_loading_position < constants::Values::kTurretRange().lower) {
        turret_loading_position =
            aos::math::NormalizeAngle(turret_loading_position);
      }

      turret_goal_buffer.Finish(
          frc971::control_loops::
              CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                  *turret_goal_buffer.fbb(), turret_loading_position));
      turret_goal = &turret_goal_buffer.message();

      const bool turret_near_goal =
          std::abs(turret_.estimated_position() - turret_loading_position) <
          kTurretGoalThreshold;
      if (!turret_near_goal) {
        break;  // Wait for turret to reach the chosen intake
      }

      // Transfer rollers and flipper arm belt on
      transfer_roller_speed =
          (intake_state_ == IntakeState::INTAKE_FRONT_BALL
               ? constants::Values::kTransferRollerFrontVoltage()
               : constants::Values::kTransferRollerBackVoltage());
      flipper_arms_voltage = constants::Values::kFlipperFeedVoltage();

      // Ball is in catapult
      if (position->turret_beambreak()) {
        intake_state_ = IntakeState::NO_BALL;
        state_ = SuperstructureState::LOADING;
        loading_timer_ = timestamp;
      }
      break;
    }
    case SuperstructureState::LOADING: {
      flipper_arms_voltage = constants::Values::kFlipperFeedVoltage();

      // Keep feeding for kExtraLoadingTime

      // The ball should go past the turret beambreak to be loaded.
      // If we got a CAN reading not too long ago, the flippers should have
      // also stopped.
      if (position->turret_beambreak()) {
        loading_timer_ = timestamp;
      } else if (timestamp >
                 loading_timer_ + constants::Values::kExtraLoadingTime()) {
        state_ = SuperstructureState::LOADED;
        reseating_in_catapult_ = false;
      }
      break;
    }
    case SuperstructureState::LOADED: {
      if (unsafe_goal != nullptr) {
        if (unsafe_goal->cancel_shot()) {
          // Cancel the shot process
          state_ = SuperstructureState::IDLE;
        } else if (unsafe_goal->fire()) {
          // Start if we were asked to and the turret is at goal
          state_ = SuperstructureState::SHOOTING;
          prev_shot_count_ = catapult_.shot_count();

          // Reset opening timeout
          flipper_opening_start_time_ = timestamp;
        }
      }
      break;
    }
    case SuperstructureState::SHOOTING: {
      // Opening flipper arms could fail, wait until they are open using their
      // potentiometers (the member below is just named encoder).
      // Be a little more lenient if the flippers were already open in case of
      // noise or collisions.
      const double flipper_open_position =
          (flippers_open_ ? constants::Values::kReseatFlipperPosition()
                          : constants::Values::kFlipperOpenPosition());

      // TODO(milind): add left arm back once it's fixed
      flippers_open_ =
          position->flipper_arm_right()->encoder() >= flipper_open_position;

      if (flippers_open_) {
        // Hold at kFlipperHoldVoltage
        flipper_arms_voltage = constants::Values::kFlipperHoldVoltage();
      } else {
        // Open at kFlipperOpenVoltage
        flipper_arms_voltage = constants::Values::kFlipperOpenVoltage();
      }

      if (!flippers_open_ &&
          timestamp >
              loading_timer_ + constants::Values::kFlipperOpeningTimeout()) {
        // Reseat the ball and try again
        state_ = SuperstructureState::LOADING;
        loading_timer_ = timestamp;
        reseating_in_catapult_ = true;
        break;
      }

      const bool turret_near_goal =
          turret_goal != nullptr &&
          std::abs(turret_goal->unsafe_goal() - turret_.position()) <
              kTurretGoalThreshold;
      const bool collided = collision_avoidance_.IsCollided(
          {.intake_front_position = intake_front_.estimated_position(),
           .intake_back_position = intake_back_.estimated_position(),
           .turret_position = turret_.estimated_position(),
           .shooting = true});

      // If the turret reached the aiming goal and the catapult is safe to move
      // up, fire!
      if (flippers_open_ && turret_near_goal && !collided) {
        fire_ = true;
      }

      // If we started firing and the flippers closed a bit, estop to prevent
      // damage
      if (fire_ && !flippers_open_) {
        catapult_.Estop();
      }

      const bool near_return_position =
          (unsafe_goal != nullptr && unsafe_goal->has_catapult() &&
           unsafe_goal->catapult()->has_return_position() &&
           std::abs(unsafe_goal->catapult()->return_position()->unsafe_goal() -
                    catapult_.estimated_position()) < kCatapultGoalThreshold);

      // Once the shot is complete and the catapult is back to its return
      // position, go back to IDLE
      if (catapult_.shot_count() > prev_shot_count_ && near_return_position) {
        prev_shot_count_ = catapult_.shot_count();
        fire_ = false;
        state_ = SuperstructureState::IDLE;
      }

      break;
    }
  }

  collision_avoidance_.UpdateGoal(
      {.intake_front_position = intake_front_.estimated_position(),
       .intake_back_position = intake_back_.estimated_position(),
       .turret_position = turret_.estimated_position(),
       .shooting = state_ == SuperstructureState::SHOOTING},
      turret_goal);

  turret_.set_min_position(collision_avoidance_.min_turret_goal());
  turret_.set_max_position(collision_avoidance_.max_turret_goal());
  intake_front_.set_min_position(collision_avoidance_.min_intake_front_goal());
  intake_front_.set_max_position(collision_avoidance_.max_intake_front_goal());
  intake_back_.set_min_position(collision_avoidance_.min_intake_back_goal());
  intake_back_.set_max_position(collision_avoidance_.max_intake_back_goal());

  const flatbuffers::Offset<AimerStatus> aimer_offset =
      aimer_.PopulateStatus(status->fbb());

  // Disable the catapult if we want to restart to prevent damage with
  // flippers
  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      catapult_status_offset =
          catapult_.Iterate(unsafe_goal, position,
                            output != nullptr && !catapult_.estopped()
                                ? &(output_struct.catapult_voltage)
                                : nullptr,
                            fire_, status->fbb());

  const flatbuffers::Offset<RelativeEncoderProfiledJointStatus>
      climber_status_offset = climber_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->climber() : nullptr,
          position->climber(),
          output != nullptr ? &output_struct.climber_voltage : nullptr,
          status->fbb());

  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      intake_status_offset_front = intake_front_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->intake_front() : nullptr,
          position->intake_front(),
          output != nullptr ? &output_struct.intake_voltage_front : nullptr,
          status->fbb());

  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      intake_status_offset_back = intake_back_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->intake_back() : nullptr,
          position->intake_back(),
          output != nullptr ? &output_struct.intake_voltage_back : nullptr,
          status->fbb());

  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      turret_status_offset = turret_.Iterate(
          turret_goal, position->turret(),
          output != nullptr ? &output_struct.turret_voltage : nullptr,
          status->fbb());

  if (output != nullptr) {
    output_struct.roller_voltage_front = roller_speed_compensated_front;
    output_struct.roller_voltage_back = roller_speed_compensated_back;
    output_struct.transfer_roller_voltage = transfer_roller_speed;
    output_struct.flipper_arms_voltage = flipper_arms_voltage;

    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();

  const bool zeroed = intake_front_.zeroed() && intake_back_.zeroed() &&
                      turret_.zeroed() && climber_.zeroed() &&
                      catapult_.zeroed();
  const bool estopped = intake_front_.estopped() || intake_back_.estopped() ||
                        turret_.estopped() || climber_.estopped() ||
                        catapult_.estopped();

  status_builder.add_zeroed(zeroed);
  status_builder.add_estopped(estopped);

  status_builder.add_intake_front(intake_status_offset_front);
  status_builder.add_intake_back(intake_status_offset_back);
  status_builder.add_turret(turret_status_offset);
  status_builder.add_climber(climber_status_offset);

  status_builder.add_catapult(catapult_status_offset);
  status_builder.add_solve_time(catapult_.solve_time());
  status_builder.add_shot_count(catapult_.shot_count());
  status_builder.add_mpc_active(catapult_.mpc_active());

  status_builder.add_flippers_open(flippers_open_);
  status_builder.add_reseating_in_catapult(reseating_in_catapult_);
  status_builder.add_fire(fire_);
  status_builder.add_state(state_);
  status_builder.add_intake_state(intake_state_);

  status_builder.add_aimer(aimer_offset);

  (void)status->Send(status_builder.Finish());
}

double Superstructure::robot_velocity() const {
  return (drivetrain_status_fetcher_.get() != nullptr
              ? drivetrain_status_fetcher_->robot_speed()
              : 0.0);
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022
