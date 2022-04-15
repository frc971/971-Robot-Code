#include "y2022/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "frc971/zeroing/wrap.h"
#include "y2022/control_loops/superstructure/collision_avoidance.h"

DEFINE_bool(ignore_distance, false,
            "If true, ignore distance when shooting and obay joystick_reader");

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
          event_loop->MakeFetcher<CANPosition>("/superstructure")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      ball_color_fetcher_(event_loop->MakeFetcher<y2022::vision::BallColor>(
          "/superstructure")),
      aimer_(values) {
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
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal, 512>
      turret_loading_goal_buffer;
  aos::FlatbufferFixedAllocatorArray<CatapultGoal, 512> catapult_goal_buffer;
  aos::FlatbufferFixedAllocatorArray<CatapultGoal, 512>
      catapult_discarding_goal_buffer;

  const aos::monotonic_clock::time_point timestamp =
      event_loop()->context().monotonic_event_time;

  if (joystick_state_fetcher_.Fetch() &&
      joystick_state_fetcher_->has_alliance()) {
    alliance_ = joystick_state_fetcher_->alliance();
  }

  if (ball_color_fetcher_.Fetch() && ball_color_fetcher_->has_ball_color()) {
    ball_color_ = ball_color_fetcher_->ball_color();
  }

  if (alliance_ != aos::Alliance::kInvalid &&
      ball_color_ != aos::Alliance::kInvalid && alliance_ != ball_color_) {
    switch (state_) {
      case SuperstructureState::IDLE:
        break;
      case SuperstructureState::TRANSFERRING:
        break;
      case SuperstructureState::LOADING:
        break;
      case SuperstructureState::LOADED:
        discarding_ball_ = true;
        break;
      case SuperstructureState::SHOOTING:
        if (!fire_) {
          // we can still tell it not to shoot into the hub
          // and change the turret and catapult goals
          discarding_ball_ = true;
        }
        break;
    }
  }

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
  const CatapultGoal *catapult_goal = nullptr;
  double roller_speed_compensated_front = 0.0;
  double roller_speed_compensated_back = 0.0;
  double transfer_roller_speed = 0.0;
  double flipper_arms_voltage = 0.0;
  bool have_active_intake_request = false;
  bool climber_servo = false;

  if (unsafe_goal != nullptr) {
    roller_speed_compensated_front =
        unsafe_goal->roller_speed_front() +
        std::max(velocity * unsafe_goal->roller_speed_compensation(), 0.0);

    roller_speed_compensated_back =
        unsafe_goal->roller_speed_back() -
        std::min(velocity * unsafe_goal->roller_speed_compensation(), 0.0);

    transfer_roller_speed = unsafe_goal->transfer_roller_speed();

    climber_servo = unsafe_goal->climber_servo();

    turret_goal = unsafe_goal->auto_aim() && !discarding_ball_
                      ? auto_aim_goal
                      : unsafe_goal->turret();

    catapult_goal = unsafe_goal->catapult();

    constants::Values::ShotParams shot_params;
    const double distance_to_goal = aimer_.DistanceToGoal();
    if (!FLAGS_ignore_distance && unsafe_goal->auto_aim() &&
        values_->shot_interpolation_table.GetInRange(distance_to_goal,
                                                     &shot_params)) {
      flatbuffers::FlatBufferBuilder *catapult_goal_fbb =
          catapult_goal_buffer.fbb();
      std::optional<flatbuffers::Offset<
          frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal>>
          return_position_offset;
      if (unsafe_goal->has_catapult() &&
          unsafe_goal->catapult()->has_return_position()) {
        return_position_offset = {aos::CopyFlatBuffer(
            unsafe_goal->catapult()->return_position(), catapult_goal_fbb)};
      }
      CatapultGoal::Builder builder(*catapult_goal_fbb);
      builder.add_shot_position(shot_params.shot_angle);
      builder.add_shot_velocity(shot_params.shot_velocity);
      if (return_position_offset.has_value()) {
        builder.add_return_position(return_position_offset.value());
      }
      catapult_goal_buffer.Finish(builder.Finish());
      catapult_goal = &catapult_goal_buffer.message();
    }

    if (discarding_ball_) {
      flatbuffers::FlatBufferBuilder *catapult_goal_fbb =
          catapult_discarding_goal_buffer.fbb();
      std::optional<flatbuffers::Offset<
          frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal>>
          return_position_offset;
      if (unsafe_goal->has_catapult() &&
          unsafe_goal->catapult()->has_return_position()) {
        return_position_offset = {aos::CopyFlatBuffer(
            unsafe_goal->catapult()->return_position(), catapult_goal_fbb)};
      }
      CatapultGoal::Builder builder(*catapult_goal_fbb);
      builder.add_shot_position(kDiscardingPosition);
      builder.add_shot_velocity(kDiscardingVelocity);
      if (return_position_offset.has_value()) {
        builder.add_return_position(return_position_offset.value());
      }
      catapult_discarding_goal_buffer.Finish(builder.Finish());
      catapult_goal = &catapult_discarding_goal_buffer.message();
    }

    if (unsafe_goal->has_turret_intake()) {
      have_active_intake_request = true;
    }
  }

  // Superstructure state machine:
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

  // If we started off preloaded, skip to the loaded state.
  // Make sure we weren't already there just in case.
  if (unsafe_goal != nullptr && unsafe_goal->preloaded()) {
    switch (state_) {
      case SuperstructureState::IDLE:
      case SuperstructureState::TRANSFERRING:
      case SuperstructureState::LOADING:
        state_ = SuperstructureState::LOADED;
        loading_timer_ = timestamp;
        break;
      case SuperstructureState::LOADED:
      case SuperstructureState::SHOOTING:
        break;
    }
  }

  if (position->intake_beambreak_front()) {
    front_intake_has_ball_ = true;
    front_intake_beambreak_timer_ = timestamp;
  }

  if (position->intake_beambreak_back()) {
    back_intake_has_ball_ = true;
    back_intake_beambreak_timer_ = timestamp;
  }

  // Check if we're either spitting or have lost the ball.
  if ((transfer_roller_speed < 0.0 && front_intake_has_ball_) ||
      timestamp >
          front_intake_beambreak_timer_ + constants::Values::kBallLostTime()) {
    front_intake_has_ball_ = false;
  }

  if ((transfer_roller_speed > 0.0 && back_intake_has_ball_) ||
      timestamp >
          back_intake_beambreak_timer_ + constants::Values::kBallLostTime()) {
    back_intake_has_ball_ = false;
  }

  // Wiggle transfer rollers: send the ball back and forth while waiting
  // for the turret or waiting for another shot to be completed
  const double wiggle_voltage =
      constants::Values::kTransferRollerWiggleVoltage();
  if (front_intake_has_ball_) {
    roller_speed_compensated_front = 0.0;
    if (position->intake_beambreak_front()) {
      transfer_roller_speed = -wiggle_voltage;
    } else {
      transfer_roller_speed = wiggle_voltage;
    }
  }

  if (back_intake_has_ball_) {
    roller_speed_compensated_back = 0.0;
    if (position->intake_beambreak_back()) {
      transfer_roller_speed = wiggle_voltage;
    } else {
      transfer_roller_speed = -wiggle_voltage;
    }
  }

  // Create the goal message for putting the turret in its loading position.
  // This will then get used in the state machine for the states (IDLE and
  // TRANSFERRING) that use it.
  double turret_loading_position =
      (turret_intake_state_ == RequestedIntake::kFront
           ? constants::Values::kTurretFrontIntakePos()
           : constants::Values::kTurretBackIntakePos());
  // Turn to the loading position as close to the middle of the range as
  // possible. Do the unwraping before we have a ball so we don't have to unwrap
  // to shoot.
  turret_loading_position =
      frc971::zeroing::Wrap(constants::Values::kTurretRange().middle_soft(),
                            turret_loading_position, 2.0 * M_PI);

  turret_loading_goal_buffer.Finish(
      frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *turret_loading_goal_buffer.fbb(), turret_loading_position));

  const bool catapult_near_return_position =
      (unsafe_goal != nullptr && unsafe_goal->has_catapult() &&
       unsafe_goal->catapult()->has_return_position() &&
       std::abs(unsafe_goal->catapult()->return_position()->unsafe_goal() -
                catapult_.estimated_position()) < kCatapultGoalThreshold);

  const bool turret_near_goal =
      turret_goal != nullptr &&
      std::abs(turret_goal->unsafe_goal() - turret_.position()) <
          kTurretGoalThreshold;
  const bool collided = collision_avoidance_.IsCollided(
      {.intake_front_position = intake_front_.estimated_position(),
       .intake_back_position = intake_back_.estimated_position(),
       .turret_position = turret_.estimated_position(),
       .shooting = true});

  // Dont shoot if the robot is moving faster than this
  constexpr double kMaxShootSpeed = 2.7;
  const bool moving_too_fast = std::abs(robot_velocity()) > kMaxShootSpeed;

  switch (state_) {
    case SuperstructureState::IDLE: {
      // Only change the turret's goal loading position when idle, to prevent us
      // spinning the turret around when TRANSFERRING...
      if (have_active_intake_request) {
        turret_intake_state_ = unsafe_goal->turret_intake();
      }
      if (front_intake_has_ball_ != back_intake_has_ball_) {
        turret_intake_state_ = front_intake_has_ball_ ? RequestedIntake::kFront
                                                      : RequestedIntake::kBack;
      }
      // When IDLE with no specific intake button pressed, allow the goal
      // message to override the intaking stuff.
      if (have_active_intake_request || (turret_goal == nullptr)) {
        turret_goal = &turret_loading_goal_buffer.message();
      }

      if (!front_intake_has_ball_ && !back_intake_has_ball_) {
        last_shot_angle_ = std::nullopt;
        break;
      }

      state_ = SuperstructureState::TRANSFERRING;
      // Save the side the ball is on for later

      break;
    }
    case SuperstructureState::TRANSFERRING: {
      // If we've been transferring for too long, the ball probably got lost.
      if ((turret_intake_state_ == RequestedIntake::kFront &&
           !front_intake_has_ball_) ||
          (turret_intake_state_ == RequestedIntake::kBack &&
           !back_intake_has_ball_)) {
        state_ = SuperstructureState::IDLE;
        break;
      }

      turret_goal = &turret_loading_goal_buffer.message();

      const bool turret_near_goal =
          std::abs(turret_.estimated_position() - turret_loading_position) <
          kTurretGoalLoadingThreshold;
      if (!turret_near_goal || !catapult_near_return_position) {
        break;  // Wait for turret to reach the chosen intake
      }

      // Transfer rollers and flipper arm belt on
      if (turret_intake_state_ == RequestedIntake::kFront) {
        transfer_roller_speed = constants::Values::kTransferRollerVoltage();
      } else {
        transfer_roller_speed = -constants::Values::kTransferRollerVoltage();
      }
      flipper_arms_voltage = constants::Values::kFlipperFeedVoltage();

      // Ball is in catapult
      if (position->turret_beambreak()) {
        if (turret_intake_state_ == RequestedIntake::kFront) {
          front_intake_has_ball_ = false;
        } else {
          back_intake_has_ball_ = false;
        }
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
        // reset color and wait for a new one once we know the ball is in place
        ball_color_ = aos::Alliance::kInvalid;
        reseating_in_catapult_ = false;
      }
      break;
    }
    case SuperstructureState::LOADED: {
      if (unsafe_goal != nullptr) {
        if (turret_goal == nullptr) {
          if (last_shot_angle_) {
            turret_loading_goal_buffer.mutable_message()->mutate_unsafe_goal(
                *last_shot_angle_);
          }
          turret_goal = &turret_loading_goal_buffer.message();
        }

        if (unsafe_goal->cancel_shot()) {
          // Cancel the shot process
          state_ = SuperstructureState::IDLE;
        } else if (unsafe_goal->fire() || discarding_ball_) {
          // Start if we were asked to and the turret is at goal
          state_ = SuperstructureState::SHOOTING;
          prev_shot_count_ = catapult_.shot_count();

          // Reset opening timeout
          flipper_opening_start_time_ = timestamp;
          loading_timer_ = timestamp;
        }
      }
      break;
    }
    case SuperstructureState::SHOOTING: {
      if (turret_goal == nullptr) {
        if (last_shot_angle_) {
          turret_loading_goal_buffer.mutable_message()->mutate_unsafe_goal(
              *last_shot_angle_);
        }
        turret_goal = &turret_loading_goal_buffer.message();
        last_shot_angle_ = turret_goal->unsafe_goal();
      } else {
        last_shot_angle_ = std::nullopt;
      }
      const bool turret_near_goal =
          turret_goal != nullptr &&
          std::abs(turret_goal->unsafe_goal() - turret_.position()) <
              kTurretGoalThreshold;

      // Don't open the flippers until the turret's ready: give them as little
      // time to get bumped as possible. Or moving to fast.
      if (!turret_near_goal || collided || moving_too_fast) {
        break;
      }

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

      // There are two possible failures for the flippers:
      // 1. They never open on time
      // 2. They opened and we started firing, but we got bumped or something
      // and they went back.
      // If the flippers didn't open in a reasonable amount of time, try
      // reseating the ball and reversing them.
      if (!fire_ && !flippers_open_ &&
          timestamp >
              loading_timer_ + constants::Values::kFlipperOpeningTimeout()) {
        // Reseat the ball and try again
        state_ = SuperstructureState::LOADING;
        loading_timer_ = timestamp;
        reseating_in_catapult_ = true;
        break;
      }

      // If the turret reached the aiming goal and the catapult is safe to move
      // up, fire!
      if (flippers_open_ && turret_near_goal && !collided) {
        fire_ = true;
      }

      // Once the shot is complete and the catapult is back to its return
      // position, go back to IDLE
      if (catapult_.shot_count() > prev_shot_count_ ) {
        prev_shot_count_ = catapult_.shot_count();
        fire_ = false;
        discarding_ball_ = false;
        state_ = SuperstructureState::IDLE;
      }

      break;
    }
  }

  collision_avoidance_.UpdateGoal(
      {.intake_front_position = intake_front_.estimated_position(),
       .intake_back_position = intake_back_.estimated_position(),
       .turret_position = turret_.estimated_position(),
       .shooting = (state_ == SuperstructureState::SHOOTING) ||
                   !catapult_near_return_position},
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
      catapult_status_offset = catapult_.Iterate(
          catapult_goal, position, robot_state().voltage_battery(),
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
    if (climber_servo) {
      output_struct.climber_servo_left = 0.5;
      output_struct.climber_servo_right = 1.0;
    } else {
      output_struct.climber_servo_left = 1.0;
      output_struct.climber_servo_right = 0.0;
    }

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
  status_builder.add_mpc_horizon(catapult_.mpc_horizon());
  if (catapult_goal != nullptr) {
    status_builder.add_shot_position(catapult_goal->shot_position());
    status_builder.add_shot_velocity(catapult_goal->shot_velocity());
  }

  status_builder.add_flippers_open(flippers_open_);
  status_builder.add_reseating_in_catapult(reseating_in_catapult_);
  status_builder.add_fire(fire_);
  status_builder.add_moving_too_fast(moving_too_fast);
  status_builder.add_discarding_ball(discarding_ball_);
  status_builder.add_ready_to_fire(state_ == SuperstructureState::LOADED &&
                                   turret_near_goal && !collided);
  status_builder.add_state(state_);
  if (!front_intake_has_ball_ && !back_intake_has_ball_) {
    status_builder.add_intake_state(IntakeState::NO_BALL);
  } else if (front_intake_has_ball_ && back_intake_has_ball_) {
    status_builder.add_intake_state(turret_intake_state_ ==
                                            RequestedIntake::kFront
                                        ? IntakeState::INTAKE_FRONT_BALL
                                        : IntakeState::INTAKE_BACK_BALL);
  } else {
    status_builder.add_intake_state(front_intake_has_ball_
                                        ? IntakeState::INTAKE_FRONT_BALL
                                        : IntakeState::INTAKE_BACK_BALL);
  }
  status_builder.add_front_intake_has_ball(front_intake_has_ball_);
  status_builder.add_back_intake_has_ball(back_intake_has_ball_);

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
