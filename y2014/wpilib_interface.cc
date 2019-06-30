#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include <thread>
#include <chrono>
#include <mutex>
#include <functional>

#include "frc971/wpilib/ahal/AnalogInput.h"
#include "frc971/wpilib/ahal/Compressor.h"
#include "frc971/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "frc971/wpilib/ahal/Encoder.h"
#include "frc971/wpilib/ahal/Relay.h"
#include "frc971/wpilib/ahal/Talon.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#undef ERROR

#include "aos/events/shm-event-loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"
#include "aos/make_unique.h"
#include "aos/robot_state/robot_state.q.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/util/phased_loop.h"
#include "aos/util/wrapping_counter.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/shifter_hall_effect.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/dma_edge_counting.h"
#include "frc971/wpilib/drivetrain_writer.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/gyro_sender.h"
#include "frc971/wpilib/interrupt_edge_counting.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging.q.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "y2014/constants.h"
#include "y2014/control_loops/claw/claw.q.h"
#include "y2014/control_loops/shooter/shooter.q.h"
#include "y2014/queues/auto_mode.q.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::y2014::control_loops::ClawQueue;
using ::y2014::control_loops::ShooterQueue;
using aos::make_unique;

namespace y2014 {
namespace wpilib {

// TODO(Brian): Fix the interpretation of the result of GetRaw here and in the
// DMA stuff and then removing the * 2.0 in *_translate.
// The low bit is direction.

double drivetrain_translate(int32_t in) {
  return -static_cast<double>(in) /
         (256.0 /*cpr*/ * 4.0 /*4x*/) *
         constants::GetValues().drivetrain_encoder_ratio *
         (3.5 /*wheel diameter*/ * 2.54 / 100.0 * M_PI) * 2.0 / 2.0;
}

double drivetrain_velocity_translate(double in) {
  return (1.0 / in) / 256.0 /*cpr*/ *
         constants::GetValues().drivetrain_encoder_ratio *
         (3.5 /*wheel diameter*/ * 2.54 / 100.0 * M_PI) * 2.0 / 2.0;
}

float hall_translate(const constants::DualHallShifterHallEffect &k, float in_low,
                     float in_high) {
  const float low_ratio =
      0.5 * (in_low - static_cast<float>(k.shifter_hall_effect.low_gear_low)) /
      static_cast<float>(k.low_gear_middle - k.shifter_hall_effect.low_gear_low);
  const float high_ratio =
      0.5 +
      0.5 * (in_high - static_cast<float>(k.high_gear_middle)) /
          static_cast<float>(k.shifter_hall_effect.high_gear_high -
                             k.high_gear_middle);

  // Return low when we are below 1/2, and high when we are above 1/2.
  if (low_ratio + high_ratio < 1.0) {
    return low_ratio;
  } else {
    return high_ratio;
  }
}

double claw_translate(int32_t in) {
  return -static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) /
         (18.0 / 48.0 /*encoder gears*/) / (12.0 / 60.0 /*chain reduction*/) *
         (M_PI / 180.0) * 2.0;
}

double shooter_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) *
         16 /*sprocket teeth*/ * 0.375 /*chain pitch*/
         * (2.54 / 100.0 /*in to m*/);
}

static const double kMaximumEncoderPulsesPerSecond =
    5600.0 /* free speed RPM */ * 14.0 / 48.0 /* bottom gear reduction */ *
    18.0 / 32.0 /* big belt reduction */ * 18.0 /
    66.0 /* top gear reduction */ * 48.0 / 18.0 /* encoder gears */ /
    60.0 /* seconds / minute */ * 256.0 /* CPR */ * 4.0 /* edges / pulse */;

class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::SensorReader(event_loop),
        auto_mode_sender_(event_loop->MakeSender<::y2014::sensors::AutoMode>(
            ".y2014.sensors.auto_mode")),
        shooter_position_sender_(event_loop->MakeSender<ShooterQueue::Position>(
            ".y2014.control_loops.shooter_queue.position")),
        claw_position_sender_(event_loop->MakeSender<ClawQueue::Position>(
            ".y2014.control_loops.claw_queue.position")),
        drivetrain_position_sender_(
            event_loop->MakeSender<
                ::frc971::control_loops::DrivetrainQueue::Position>(
                ".frc971.control_loops.drivetrain_queue.position")) {
    // Set it to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    UpdateMediumEncoderFilterHz(kMaximumEncoderPulsesPerSecond);
    hall_filter_.SetPeriodNanoSeconds(100000);
  }

  ~SensorReader() override {
    top_reader_.Quit();
    bottom_reader_.Quit();
  }

  void set_auto_selector_analog(::std::unique_ptr<::frc::AnalogInput> analog) {
    auto_selector_analog_ = ::std::move(analog);
  }

  void set_high_left_drive_hall(::std::unique_ptr<::frc::AnalogInput> analog) {
    high_left_drive_hall_ = ::std::move(analog);
  }

  void set_low_right_drive_hall(::std::unique_ptr<::frc::AnalogInput> analog) {
    low_right_drive_hall_ = ::std::move(analog);
  }

  void set_high_right_drive_hall(::std::unique_ptr<::frc::AnalogInput> analog) {
    high_right_drive_hall_ = ::std::move(analog);
  }

  void set_low_left_drive_hall(::std::unique_ptr<::frc::AnalogInput> analog) {
    low_left_drive_hall_ = ::std::move(analog);
  }

  void set_top_claw_encoder(::std::unique_ptr<::frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    top_reader_.set_encoder(::std::move(encoder));
  }

  void set_top_claw_front_hall(::std::unique_ptr<::frc::DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    top_reader_.set_front_hall(::std::move(hall));
  }

  void set_top_claw_calibration_hall(
      ::std::unique_ptr<::frc::DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    top_reader_.set_calibration_hall(::std::move(hall));
  }

  void set_top_claw_back_hall(::std::unique_ptr<::frc::DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    top_reader_.set_back_hall(::std::move(hall));
  }

  void set_bottom_claw_encoder(::std::unique_ptr<::frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    bottom_reader_.set_encoder(::std::move(encoder));
  }

  void set_bottom_claw_front_hall(::std::unique_ptr<::frc::DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    bottom_reader_.set_front_hall(::std::move(hall));
  }

  void set_bottom_claw_calibration_hall(
      ::std::unique_ptr<::frc::DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    bottom_reader_.set_calibration_hall(::std::move(hall));
  }

  void set_bottom_claw_back_hall(::std::unique_ptr<::frc::DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    bottom_reader_.set_back_hall(::std::move(hall));
  }

  void set_shooter_encoder(::std::unique_ptr<::frc::Encoder> encoder) {
    medium_encoder_filter_.Add(encoder.get());
    shooter_encoder_ = ::std::move(encoder);
  }

  void set_shooter_proximal(::std::unique_ptr<::frc::DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    shooter_proximal_ = ::std::move(hall);
  }

  void set_shooter_distal(::std::unique_ptr<::frc::DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    shooter_distal_ = ::std::move(hall);
  }

  void set_shooter_plunger(::std::unique_ptr<::frc::DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    shooter_plunger_ = ::std::move(hall);
    shooter_plunger_reader_ =
        make_unique<::frc971::wpilib::DMADigitalReader>(shooter_plunger_.get());
  }

  void set_shooter_latch(::std::unique_ptr<::frc::DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    shooter_latch_ = ::std::move(hall);
    shooter_latch_reader_ =
        make_unique<::frc971::wpilib::DMADigitalReader>(shooter_latch_.get());
  }

  void Start() override {
    shooter_proximal_counter_ = make_unique<::frc971::wpilib::DMAEdgeCounter>(
        shooter_encoder_.get(), shooter_proximal_.get());
    shooter_distal_counter_ = make_unique<::frc971::wpilib::DMAEdgeCounter>(
        shooter_encoder_.get(), shooter_distal_.get());

    AddToDMA(shooter_proximal_counter_.get());
    AddToDMA(shooter_distal_counter_.get());
    AddToDMA(shooter_plunger_reader_.get());
    AddToDMA(shooter_latch_reader_.get());

    top_reader_.Start();
    bottom_reader_.Start();
  }

  void RunIteration() {
    const auto &values = constants::GetValues();

    {
      auto drivetrain_message = drivetrain_position_sender_.MakeMessage();
      drivetrain_message->right_encoder =
          drivetrain_translate(drivetrain_right_encoder_->GetRaw());
      drivetrain_message->left_encoder =
          -drivetrain_translate(drivetrain_left_encoder_->GetRaw());
      drivetrain_message->left_speed =
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod());
      drivetrain_message->right_speed =
          drivetrain_velocity_translate(drivetrain_right_encoder_->GetPeriod());

      drivetrain_message->low_left_hall = low_left_drive_hall_->GetVoltage();
      drivetrain_message->high_left_hall = high_left_drive_hall_->GetVoltage();
      drivetrain_message->left_shifter_position =
          hall_translate(values.left_drive, drivetrain_message->low_left_hall,
                         drivetrain_message->high_left_hall);

      drivetrain_message->low_right_hall = low_right_drive_hall_->GetVoltage();
      drivetrain_message->high_right_hall =
          high_right_drive_hall_->GetVoltage();
      drivetrain_message->right_shifter_position =
          hall_translate(values.right_drive, drivetrain_message->low_right_hall,
                         drivetrain_message->high_right_hall);

      drivetrain_message.Send();
    }

    {
      auto auto_mode_message = auto_mode_sender_.MakeMessage();
      auto_mode_message->voltage = auto_selector_analog_->GetVoltage();
      auto_mode_message.Send();
    }
  }

  void RunDmaIteration() {
    {
      auto shooter_message = shooter_position_sender_.MakeMessage();
      shooter_message->position = shooter_translate(shooter_encoder_->GetRaw());
      shooter_message->plunger = !shooter_plunger_reader_->value();
      shooter_message->latch = !shooter_latch_reader_->value();
      CopyShooterPosedgeCounts(shooter_proximal_counter_.get(),
                               &shooter_message->pusher_proximal);
      CopyShooterPosedgeCounts(shooter_distal_counter_.get(),
                               &shooter_message->pusher_distal);

      shooter_message.Send();
    }

    {
      auto claw_message = claw_position_sender_.MakeMessage();
      top_reader_.RunIteration(&claw_message->top);
      bottom_reader_.RunIteration(&claw_message->bottom);

      claw_message.Send();
    }
  }

 private:
  class HalfClawReader {
   public:
    HalfClawReader(bool reversed) : reversed_(reversed) {}

    void set_encoder(::std::unique_ptr<::frc::Encoder> encoder) {
      encoder_ = ::std::move(encoder);
    }

    void set_front_hall(::std::unique_ptr<::frc::DigitalInput> front_hall) {
      front_hall_ = ::std::move(front_hall);
    }

    void set_calibration_hall(
        ::std::unique_ptr<::frc::DigitalInput> calibration_hall) {
      calibration_hall_ = ::std::move(calibration_hall);
    }

    void set_back_hall(::std::unique_ptr<::frc::DigitalInput> back_hall) {
      back_hall_ = ::std::move(back_hall);
    }

    void Start() {
      front_counter_ = make_unique<::frc971::wpilib::EdgeCounter>(
          encoder_.get(), front_hall_.get());
      synchronizer_.Add(front_counter_.get());
      calibration_counter_ = make_unique<::frc971::wpilib::EdgeCounter>(
          encoder_.get(), calibration_hall_.get());
      synchronizer_.Add(calibration_counter_.get());
      back_counter_ = make_unique<::frc971::wpilib::EdgeCounter>(
          encoder_.get(), back_hall_.get());
      synchronizer_.Add(back_counter_.get());
      synchronized_encoder_ =
          make_unique<::frc971::wpilib::InterruptSynchronizedEncoder>(
              encoder_.get());
      synchronizer_.Add(synchronized_encoder_.get());

      synchronizer_.Start();
    }

    void Quit() { synchronizer_.Quit(); }

    void RunIteration(control_loops::HalfClawPosition *half_claw_position) {
      const double multiplier = reversed_ ? -1.0 : 1.0;

      synchronizer_.RunIteration();

      CopyPosition(front_counter_.get(), &half_claw_position->front);
      CopyPosition(calibration_counter_.get(),
                   &half_claw_position->calibration);
      CopyPosition(back_counter_.get(), &half_claw_position->back);
      half_claw_position->position =
          multiplier * claw_translate(synchronized_encoder_->get());
    }

   private:
    void CopyPosition(const ::frc971::wpilib::EdgeCounter *counter,
                      ::frc971::HallEffectStruct *out) {
      const double multiplier = reversed_ ? -1.0 : 1.0;

      out->current = !counter->polled_value();
      out->posedge_count = counter->negative_interrupt_count();
      out->negedge_count = counter->positive_interrupt_count();
      out->negedge_value =
          multiplier * claw_translate(counter->last_positive_encoder_value());
      out->posedge_value =
          multiplier * claw_translate(counter->last_negative_encoder_value());
    }

    ::frc971::wpilib::InterruptSynchronizer synchronizer_{55};

    ::std::unique_ptr<::frc971::wpilib::EdgeCounter> front_counter_;
    ::std::unique_ptr<::frc971::wpilib::EdgeCounter> calibration_counter_;
    ::std::unique_ptr<::frc971::wpilib::EdgeCounter> back_counter_;
    ::std::unique_ptr<::frc971::wpilib::InterruptSynchronizedEncoder>
        synchronized_encoder_;

    ::std::unique_ptr<::frc::Encoder> encoder_;
    ::std::unique_ptr<::frc::DigitalInput> front_hall_;
    ::std::unique_ptr<::frc::DigitalInput> calibration_hall_;
    ::std::unique_ptr<::frc::DigitalInput> back_hall_;

    const bool reversed_;
  };

  void CopyShooterPosedgeCounts(
      const ::frc971::wpilib::DMAEdgeCounter *counter,
      ::frc971::PosedgeOnlyCountedHallEffectStruct *output) {
    output->current = !counter->polled_value();
    // These are inverted because the hall effects give logical false when
    // there's a magnet in front of them.
    output->posedge_count = counter->negative_count();
    output->negedge_count = counter->positive_count();
    output->posedge_value =
        shooter_translate(counter->last_negative_encoder_value());
  }

  ::aos::Sender<::y2014::sensors::AutoMode> auto_mode_sender_;
  ::aos::Sender<ShooterQueue::Position> shooter_position_sender_;
  ::aos::Sender<ClawQueue::Position> claw_position_sender_;
  ::aos::Sender<::frc971::control_loops::DrivetrainQueue::Position>
      drivetrain_position_sender_;

  ::std::unique_ptr<::frc::AnalogInput> auto_selector_analog_;

  ::std::unique_ptr<::frc::AnalogInput> low_left_drive_hall_;
  ::std::unique_ptr<::frc::AnalogInput> high_left_drive_hall_;
  ::std::unique_ptr<::frc::AnalogInput> low_right_drive_hall_;
  ::std::unique_ptr<::frc::AnalogInput> high_right_drive_hall_;

  HalfClawReader top_reader_{false}, bottom_reader_{true};

  ::std::unique_ptr<::frc::Encoder> shooter_encoder_;
  ::std::unique_ptr<::frc::DigitalInput> shooter_proximal_, shooter_distal_;
  ::std::unique_ptr<::frc::DigitalInput> shooter_plunger_, shooter_latch_;
  ::std::unique_ptr<::frc971::wpilib::DMAEdgeCounter> shooter_proximal_counter_,
      shooter_distal_counter_;
  ::std::unique_ptr<::frc971::wpilib::DMADigitalReader> shooter_plunger_reader_,
      shooter_latch_reader_;

  ::frc::DigitalGlitchFilter hall_filter_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(::aos::EventLoop *event_loop,
                 const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm)
      : pcm_(pcm),
        shooter_(event_loop->MakeFetcher<ShooterQueue::Output>(
            ".y2014.control_loops.shooter_queue.output")),
        drivetrain_(
            event_loop
                ->MakeFetcher<::frc971::control_loops::DrivetrainQueue::Output>(
                    ".frc971.control_loops.drivetrain_queue.output")) {
    event_loop->set_name("Solenoids");
    event_loop->SetRuntimeRealtimePriority(27);

    event_loop->AddPhasedLoop([this](int iterations) { Loop(iterations); },
                              ::std::chrono::milliseconds(20),
                              ::std::chrono::milliseconds(1));
  }

  void set_pressure_switch(
      ::std::unique_ptr<::frc::DigitalInput> pressure_switch) {
    pressure_switch_ = ::std::move(pressure_switch);
  }

  void set_compressor_relay(::std::unique_ptr<::frc::Relay> compressor_relay) {
    compressor_relay_ = ::std::move(compressor_relay);
  }

  void set_drivetrain_left(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    drivetrain_left_ = ::std::move(s);
  }

  void set_drivetrain_right(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    drivetrain_right_ = ::std::move(s);
  }

  void set_shooter_latch(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    shooter_latch_ = ::std::move(s);
  }

  void set_shooter_brake(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    shooter_brake_ = ::std::move(s);
  }

  void Loop(const int iterations) {
    if (iterations != 1) {
      LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
    }

    {
      shooter_.Fetch();
      if (shooter_.get()) {
        LOG_STRUCT(DEBUG, "solenoids", *shooter_);
        shooter_latch_->Set(!shooter_->latch_piston);
        shooter_brake_->Set(!shooter_->brake_piston);
      }
    }

    {
      drivetrain_.Fetch();
      if (drivetrain_.get()) {
        LOG_STRUCT(DEBUG, "solenoids", *drivetrain_);
        drivetrain_left_->Set(!drivetrain_->left_high);
        drivetrain_right_->Set(!drivetrain_->right_high);
      }
    }

    {
      ::frc971::wpilib::PneumaticsToLog to_log;
      {
        const bool compressor_on = !pressure_switch_->Get();
        to_log.compressor_on = compressor_on;
        if (compressor_on) {
          compressor_relay_->Set(::frc::Relay::kForward);
        } else {
          compressor_relay_->Set(::frc::Relay::kOff);
        }
      }

      pcm_->Flush();
      to_log.read_solenoids = pcm_->GetAll();
      LOG_STRUCT(DEBUG, "pneumatics info", to_log);
    }
  }

 private:
  const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm_;

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> drivetrain_left_;
  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> drivetrain_right_;
  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> shooter_latch_;
  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> shooter_brake_;

  ::std::unique_ptr<::frc::DigitalInput> pressure_switch_;
  ::std::unique_ptr<::frc::Relay> compressor_relay_;

  ::aos::Fetcher<ShooterQueue::Output> shooter_;
  ::aos::Fetcher<::frc971::control_loops::DrivetrainQueue::Output> drivetrain_;
};

class ShooterWriter
    : public ::frc971::wpilib::LoopOutputHandler<ShooterQueue::Output> {
 public:
  ShooterWriter(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::LoopOutputHandler<ShooterQueue::Output>(
            event_loop, ".y2014.control_loops.shooter_queue.output") {}

  void set_shooter_talon(::std::unique_ptr<::frc::Talon> t) {
    shooter_talon_ = ::std::move(t);
  }

 private:
  virtual void Write(const ShooterQueue::Output &output) override {
    LOG_STRUCT(DEBUG, "will output", output);
    shooter_talon_->SetSpeed(output.voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "shooter output too old\n");
    shooter_talon_->SetDisabled();
  }

  ::std::unique_ptr<::frc::Talon> shooter_talon_;
};

class ClawWriter
    : public ::frc971::wpilib::LoopOutputHandler<ClawQueue::Output> {
 public:
  ClawWriter(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::LoopOutputHandler<ClawQueue::Output>(
            event_loop, ".y2014.control_loops.claw_queue.output") {}

  void set_top_claw_talon(::std::unique_ptr<::frc::Talon> t) {
    top_claw_talon_ = ::std::move(t);
  }

  void set_bottom_claw_talon(::std::unique_ptr<::frc::Talon> t) {
    bottom_claw_talon_ = ::std::move(t);
  }

  void set_left_tusk_talon(::std::unique_ptr<::frc::Talon> t) {
    left_tusk_talon_ = ::std::move(t);
  }

  void set_right_tusk_talon(::std::unique_ptr<::frc::Talon> t) {
    right_tusk_talon_ = ::std::move(t);
  }

  void set_intake1_talon(::std::unique_ptr<::frc::Talon> t) {
    intake1_talon_ = ::std::move(t);
  }

  void set_intake2_talon(::std::unique_ptr<::frc::Talon> t) {
    intake2_talon_ = ::std::move(t);
  }

 private:
  virtual void Write(const ClawQueue::Output &output) override {
    LOG_STRUCT(DEBUG, "will output", output);
    intake1_talon_->SetSpeed(output.intake_voltage / 12.0);
    intake2_talon_->SetSpeed(output.intake_voltage / 12.0);
    bottom_claw_talon_->SetSpeed(-output.bottom_claw_voltage / 12.0);
    top_claw_talon_->SetSpeed(output.top_claw_voltage / 12.0);
    left_tusk_talon_->SetSpeed(output.tusk_voltage / 12.0);
    right_tusk_talon_->SetSpeed(-output.tusk_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "claw output too old\n");
    intake1_talon_->SetDisabled();
    intake2_talon_->SetDisabled();
    bottom_claw_talon_->SetDisabled();
    top_claw_talon_->SetDisabled();
    left_tusk_talon_->SetDisabled();
    right_tusk_talon_->SetDisabled();
  }

  ::std::unique_ptr<::frc::Talon> top_claw_talon_;
  ::std::unique_ptr<::frc::Talon> bottom_claw_talon_;
  ::std::unique_ptr<::frc::Talon> left_tusk_talon_;
  ::std::unique_ptr<::frc::Talon> right_tusk_talon_;
  ::std::unique_ptr<::frc::Talon> intake1_talon_;
  ::std::unique_ptr<::frc::Talon> intake2_talon_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<::frc::Encoder> make_encoder(int index) {
    return make_unique<::frc::Encoder>(10 + index * 2, 11 + index * 2, false,
                                       ::frc::Encoder::k4X);
  }

  void Run() override {
    // Thread 1.
    ::aos::ShmEventLoop joystick_sender_event_loop;
    ::frc971::wpilib::JoystickSender joystick_sender(
        &joystick_sender_event_loop);
    AddLoop(&joystick_sender_event_loop);

    // Thread 2.
    ::aos::ShmEventLoop pdp_fetcher_event_loop;
    ::frc971::wpilib::PDPFetcher pdp_fetcher(&pdp_fetcher_event_loop);
    AddLoop(&pdp_fetcher_event_loop);

    // Thread 3.
    ::aos::ShmEventLoop sensor_reader_event_loop;
    SensorReader sensor_reader(&sensor_reader_event_loop);

    // Create this first to make sure it ends up in one of the lower-numbered
    // FPGA slots so we can use it with DMA.
    auto shooter_encoder_temp = make_encoder(2);

    sensor_reader.set_auto_selector_analog(make_unique<::frc::AnalogInput>(4));

    sensor_reader.set_drivetrain_left_encoder(make_encoder(0));
    sensor_reader.set_drivetrain_right_encoder(make_encoder(1));
    sensor_reader.set_high_left_drive_hall(make_unique<::frc::AnalogInput>(1));
    sensor_reader.set_low_left_drive_hall(make_unique<::frc::AnalogInput>(0));
    sensor_reader.set_high_right_drive_hall(make_unique<::frc::AnalogInput>(2));
    sensor_reader.set_low_right_drive_hall(make_unique<::frc::AnalogInput>(3));

    sensor_reader.set_top_claw_encoder(make_encoder(3));
    sensor_reader.set_top_claw_front_hall(
        make_unique<::frc::DigitalInput>(4));  // R2
    sensor_reader.set_top_claw_calibration_hall(
        make_unique<::frc::DigitalInput>(3));  // R3
    sensor_reader.set_top_claw_back_hall(
        make_unique<::frc::DigitalInput>(5));  // R1

    sensor_reader.set_bottom_claw_encoder(make_encoder(4));
    sensor_reader.set_bottom_claw_front_hall(
        make_unique<::frc::DigitalInput>(1));  // L2
    sensor_reader.set_bottom_claw_calibration_hall(
        make_unique<::frc::DigitalInput>(0));  // L3
    sensor_reader.set_bottom_claw_back_hall(
        make_unique<::frc::DigitalInput>(2));  // L1

    sensor_reader.set_shooter_encoder(::std::move(shooter_encoder_temp));
    sensor_reader.set_shooter_proximal(
        make_unique<::frc::DigitalInput>(6));  // S1
    sensor_reader.set_shooter_distal(
        make_unique<::frc::DigitalInput>(7));  // S2
    sensor_reader.set_shooter_plunger(
        make_unique<::frc::DigitalInput>(8));                              // S3
    sensor_reader.set_shooter_latch(make_unique<::frc::DigitalInput>(9));  // S4
    AddLoop(&sensor_reader_event_loop);

    // Thread 4.
    ::aos::ShmEventLoop gyro_event_loop;
    ::frc971::wpilib::GyroSender gyro_sender(&gyro_event_loop);
    AddLoop(&gyro_event_loop);

    // Thread 5.
    ::aos::ShmEventLoop output_event_loop;
    ::frc971::wpilib::DrivetrainWriter drivetrain_writer(&output_event_loop);
    drivetrain_writer.set_left_controller0(make_unique<::frc::Talon>(5), true);
    drivetrain_writer.set_right_controller0(make_unique<::frc::Talon>(2),
                                            false);

    ::y2014::wpilib::ClawWriter claw_writer(&output_event_loop);
    claw_writer.set_top_claw_talon(make_unique<::frc::Talon>(1));
    claw_writer.set_bottom_claw_talon(make_unique<::frc::Talon>(0));
    claw_writer.set_left_tusk_talon(make_unique<::frc::Talon>(4));
    claw_writer.set_right_tusk_talon(make_unique<::frc::Talon>(3));
    claw_writer.set_intake1_talon(make_unique<::frc::Talon>(7));
    claw_writer.set_intake2_talon(make_unique<::frc::Talon>(8));

    ::y2014::wpilib::ShooterWriter shooter_writer(&output_event_loop);
    shooter_writer.set_shooter_talon(make_unique<::frc::Talon>(6));

    AddLoop(&output_event_loop);

    // Thread 6.
    ::aos::ShmEventLoop solenoid_writer_event_loop;
    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(&solenoid_writer_event_loop, pcm);
    solenoid_writer.set_drivetrain_left(pcm->MakeSolenoid(6));
    solenoid_writer.set_drivetrain_right(pcm->MakeSolenoid(7));
    solenoid_writer.set_shooter_latch(pcm->MakeSolenoid(5));
    solenoid_writer.set_shooter_brake(pcm->MakeSolenoid(4));

    solenoid_writer.set_pressure_switch(make_unique<::frc::DigitalInput>(25));
    solenoid_writer.set_compressor_relay(make_unique<::frc::Relay>(0));
    AddLoop(&solenoid_writer_event_loop);

    RunLoops();
  }
};

}  // namespace wpilib
}  // namespace y2014

AOS_ROBOT_CLASS(::y2014::wpilib::WPILibRobot);
