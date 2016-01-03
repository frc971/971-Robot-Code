#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include <thread>
#include <mutex>
#include <functional>

#include "Encoder.h"
#include "Talon.h"
#include "DriverStation.h"
#include "AnalogInput.h"
#include "Compressor.h"
#include "Relay.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "dma.h"
#ifndef WPILIB2015
#include "DigitalGlitchFilter.h"
#endif
#undef ERROR

#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/time.h"
#include "aos/common/util/log_interval.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/util/wrapping_counter.h"
#include "aos/common/stl_mutex.h"
#include "aos/linux_code/init.h"
#include "aos/common/messages/robot_state.q.h"

#include "frc971/shifter_hall_effect.h"

#include "y2014/control_loops/drivetrain/drivetrain.q.h"
#include "y2014/control_loops/claw/claw.q.h"
#include "y2014/control_loops/shooter/shooter.q.h"
#include "y2014/constants.h"
#include "y2014/queues/auto_mode.q.h"

#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/gyro_sender.h"
#include "frc971/wpilib/dma_edge_counting.h"
#include "frc971/wpilib/interrupt_edge_counting.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/logging.q.h"
#include "frc971/wpilib/wpilib_interface.h"
#include "frc971/wpilib/pdp_fetcher.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::y2014::control_loops::drivetrain_queue;
using ::y2014::control_loops::claw_queue;
using ::y2014::control_loops::shooter_queue;

namespace y2014 {
namespace wpilib {

// TODO(Brian): Fix the interpretation of the result of GetRaw here and in the
// DMA stuff and then removing the * 2.0 in *_translate.
// The low bit is direction.

// TODO(brian): Replace this with ::std::make_unique once all our toolchains
// have support.
template <class T, class... U>
std::unique_ptr<T> make_unique(U &&... u) {
  return std::unique_ptr<T>(new T(std::forward<U>(u)...));
}

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

float hall_translate(const constants::ShifterHallEffect &k, float in_low,
                     float in_high) {
  const float low_ratio =
      0.5 * (in_low - static_cast<float>(k.low_gear_low)) /
      static_cast<float>(k.low_gear_middle - k.low_gear_low);
  const float high_ratio =
      0.5 + 0.5 * (in_high - static_cast<float>(k.high_gear_middle)) /
                static_cast<float>(k.high_gear_high - k.high_gear_middle);

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
    18.0 / 32.0 /* big belt reduction */ *
    18.0 / 66.0 /* top gear reduction */ * 48.0 / 18.0 /* encoder gears */ /
    60.0 /* seconds / minute */ * 256.0 /* CPR */;

class SensorReader {
 public:
  SensorReader() {
    // Set it to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    encoder_filter_.SetPeriodNanoSeconds(
        static_cast<int>(1 / 4.0 / kMaximumEncoderPulsesPerSecond * 1e9 + 0.5));
    hall_filter_.SetPeriodNanoSeconds(100000);
  }

  void set_auto_selector_analog(::std::unique_ptr<AnalogInput> analog) {
    auto_selector_analog_ = ::std::move(analog);
  }

  void set_drivetrain_left_encoder(::std::unique_ptr<Encoder> encoder) {
    drivetrain_left_encoder_ = ::std::move(encoder);
    drivetrain_left_encoder_->SetMaxPeriod(0.005);
  }

  void set_drivetrain_right_encoder(::std::unique_ptr<Encoder> encoder) {
    drivetrain_right_encoder_ = ::std::move(encoder);
    drivetrain_right_encoder_->SetMaxPeriod(0.005);
  }

  void set_high_left_drive_hall(::std::unique_ptr<AnalogInput> analog) {
    high_left_drive_hall_ = ::std::move(analog);
  }

  void set_low_right_drive_hall(::std::unique_ptr<AnalogInput> analog) {
    low_right_drive_hall_ = ::std::move(analog);
  }

  void set_high_right_drive_hall(::std::unique_ptr<AnalogInput> analog) {
    high_right_drive_hall_ = ::std::move(analog);
  }

  void set_low_left_drive_hall(::std::unique_ptr<AnalogInput> analog) {
    low_left_drive_hall_ = ::std::move(analog);
  }

  void set_top_claw_encoder(::std::unique_ptr<Encoder> encoder) {
    encoder_filter_.Add(encoder.get());
    top_reader_.set_encoder(::std::move(encoder));
  }

  void set_top_claw_front_hall(::std::unique_ptr<DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    top_reader_.set_front_hall(::std::move(hall));
  }

  void set_top_claw_calibration_hall(::std::unique_ptr<DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    top_reader_.set_calibration_hall(::std::move(hall));
  }

  void set_top_claw_back_hall(::std::unique_ptr<DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    top_reader_.set_back_hall(::std::move(hall));
  }

  void set_bottom_claw_encoder(::std::unique_ptr<Encoder> encoder) {
    encoder_filter_.Add(encoder.get());
    bottom_reader_.set_encoder(::std::move(encoder));
  }

  void set_bottom_claw_front_hall(::std::unique_ptr<DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    bottom_reader_.set_front_hall(::std::move(hall));
  }

  void set_bottom_claw_calibration_hall(::std::unique_ptr<DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    bottom_reader_.set_calibration_hall(::std::move(hall));
  }

  void set_bottom_claw_back_hall(::std::unique_ptr<DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    bottom_reader_.set_back_hall(::std::move(hall));
  }

  void set_shooter_encoder(::std::unique_ptr<Encoder> encoder) {
    encoder_filter_.Add(encoder.get());
    shooter_encoder_ = ::std::move(encoder);
  }

  void set_shooter_proximal(::std::unique_ptr<DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    shooter_proximal_ = ::std::move(hall);
  }

  void set_shooter_distal(::std::unique_ptr<DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    shooter_distal_ = ::std::move(hall);
  }

  void set_shooter_plunger(::std::unique_ptr<DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    shooter_plunger_ = ::std::move(hall);
    shooter_plunger_reader_ =
        make_unique<::frc971::wpilib::DMADigitalReader>(shooter_plunger_.get());
  }

  void set_shooter_latch(::std::unique_ptr<DigitalInput> hall) {
    hall_filter_.Add(hall.get());
    shooter_latch_ = ::std::move(hall);
    shooter_latch_reader_ =
        make_unique<::frc971::wpilib::DMADigitalReader>(shooter_latch_.get());
  }

  // All of the DMA-related set_* calls must be made before this, and it doesn't
  // hurt to do all of them.
  void set_dma(::std::unique_ptr<DMA> dma) {
    shooter_proximal_counter_ = make_unique<::frc971::wpilib::DMAEdgeCounter>(
        shooter_encoder_.get(), shooter_proximal_.get());
    shooter_distal_counter_ = make_unique<::frc971::wpilib::DMAEdgeCounter>(
        shooter_encoder_.get(), shooter_distal_.get());

    dma_synchronizer_.reset(
        new ::frc971::wpilib::DMASynchronizer(::std::move(dma)));
    dma_synchronizer_->Add(shooter_proximal_counter_.get());
    dma_synchronizer_->Add(shooter_distal_counter_.get());
    dma_synchronizer_->Add(shooter_plunger_reader_.get());
    dma_synchronizer_->Add(shooter_latch_reader_.get());
  }

  void operator()() {
    ::aos::SetCurrentThreadName("SensorReader");

    my_pid_ = getpid();
    ds_ =
#ifdef WPILIB2015
        DriverStation::GetInstance();
#else
        &DriverStation::GetInstance();
#endif

    top_reader_.Start();
    bottom_reader_.Start();
    dma_synchronizer_->Start();

    ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                        ::aos::time::Time::InMS(4));

    ::aos::SetCurrentThreadRealtimePriority(40);
    while (run_) {
      {
        const int iterations = phased_loop.SleepUntilNext();
        if (iterations != 1) {
          LOG(WARNING, "SensorReader skipped %d iterations\n", iterations - 1);
        }
      }
      RunIteration();
    }

    top_reader_.Quit();
    bottom_reader_.Quit();
  }

  void RunIteration() {
    ::frc971::wpilib::SendRobotState(my_pid_, ds_);

    const auto &values = constants::GetValues();

    {
      auto drivetrain_message = drivetrain_queue.position.MakeMessage();
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

    ::y2014::sensors::auto_mode.MakeWithBuilder()
        .voltage(auto_selector_analog_->GetVoltage())
        .Send();

    dma_synchronizer_->RunIteration();

    {
      auto shooter_message = shooter_queue.position.MakeMessage();
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
      auto claw_message = claw_queue.position.MakeMessage();
      top_reader_.RunIteration(&claw_message->top);
      bottom_reader_.RunIteration(&claw_message->bottom);

      claw_message.Send();
    }
  }

  void Quit() { run_ = false; }

 private:
  class HalfClawReader {
   public:
    HalfClawReader(bool reversed) : reversed_(reversed) {}

    void set_encoder(::std::unique_ptr<Encoder> encoder) {
      encoder_ = ::std::move(encoder);
    }

    void set_front_hall(::std::unique_ptr<DigitalInput> front_hall) {
      front_hall_ = ::std::move(front_hall);
    }

    void set_calibration_hall(
        ::std::unique_ptr<DigitalInput> calibration_hall) {
      calibration_hall_ = ::std::move(calibration_hall);
    }

    void set_back_hall(::std::unique_ptr<DigitalInput> back_hall) {
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

    ::std::unique_ptr<Encoder> encoder_;
    ::std::unique_ptr<DigitalInput> front_hall_;
    ::std::unique_ptr<DigitalInput> calibration_hall_;
    ::std::unique_ptr<DigitalInput> back_hall_;

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

  int32_t my_pid_;
  DriverStation *ds_;

  ::std::unique_ptr<::frc971::wpilib::DMASynchronizer> dma_synchronizer_;

  ::std::unique_ptr<AnalogInput> auto_selector_analog_;

  ::std::unique_ptr<Encoder> drivetrain_left_encoder_;
  ::std::unique_ptr<Encoder> drivetrain_right_encoder_;
  ::std::unique_ptr<AnalogInput> low_left_drive_hall_;
  ::std::unique_ptr<AnalogInput> high_left_drive_hall_;
  ::std::unique_ptr<AnalogInput> low_right_drive_hall_;
  ::std::unique_ptr<AnalogInput> high_right_drive_hall_;

  HalfClawReader top_reader_{false}, bottom_reader_{true};

  ::std::unique_ptr<Encoder> shooter_encoder_;
  ::std::unique_ptr<DigitalInput> shooter_proximal_, shooter_distal_;
  ::std::unique_ptr<DigitalInput> shooter_plunger_, shooter_latch_;
  ::std::unique_ptr<::frc971::wpilib::DMAEdgeCounter> shooter_proximal_counter_,
      shooter_distal_counter_;
  ::std::unique_ptr<::frc971::wpilib::DMADigitalReader> shooter_plunger_reader_,
      shooter_latch_reader_;

  ::std::atomic<bool> run_{true};
  DigitalGlitchFilter encoder_filter_, hall_filter_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm)
      : pcm_(pcm),
        shooter_(".y2014.control_loops.shooter_queue.output"),
        drivetrain_(".y2014.control_loops.drivetrain_queue.output") {}

  void set_pressure_switch(::std::unique_ptr<DigitalInput> pressure_switch) {
    pressure_switch_ = ::std::move(pressure_switch);
  }

  void set_compressor_relay(::std::unique_ptr<Relay> compressor_relay) {
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

  void operator()() {
    ::aos::SetCurrentThreadName("Solenoids");
    ::aos::SetCurrentThreadRealtimePriority(27);

    ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(20),
                                        ::aos::time::Time::InMS(1));

    while (run_) {
      {
        const int iterations = phased_loop.SleepUntilNext();
        if (iterations != 1) {
          LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
        }
      }

      {
        shooter_.FetchLatest();
        if (shooter_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *shooter_);
          shooter_latch_->Set(!shooter_->latch_piston);
          shooter_brake_->Set(!shooter_->brake_piston);
        }
      }

      {
        drivetrain_.FetchLatest();
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
            compressor_relay_->Set(Relay::kForward);
          } else {
            compressor_relay_->Set(Relay::kOff);
          }
        }

        pcm_->Flush();
        to_log.read_solenoids = pcm_->GetAll();
        LOG_STRUCT(DEBUG, "pneumatics info", to_log);
      }
    }
  }

  void Quit() { run_ = false; }

 private:
  const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm_;

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> drivetrain_left_;
  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> drivetrain_right_;
  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> shooter_latch_;
  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> shooter_brake_;

  ::std::unique_ptr<DigitalInput> pressure_switch_;
  ::std::unique_ptr<Relay> compressor_relay_;

  ::aos::Queue<::y2014::control_loops::ShooterQueue::Output> shooter_;
  ::aos::Queue<::y2014::control_loops::DrivetrainQueue::Output> drivetrain_;

  ::std::atomic<bool> run_{true};
};

class DrivetrainWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_left_drivetrain_talon(::std::unique_ptr<Talon> t) {
    left_drivetrain_talon_ = ::std::move(t);
  }

  void set_right_drivetrain_talon(::std::unique_ptr<Talon> t) {
    right_drivetrain_talon_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2014::control_loops::drivetrain_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2014::control_loops::drivetrain_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    left_drivetrain_talon_->Set(-queue->left_voltage / 12.0);
    right_drivetrain_talon_->Set(queue->right_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "drivetrain output too old\n");
    left_drivetrain_talon_->Disable();
    right_drivetrain_talon_->Disable();
  }

  ::std::unique_ptr<Talon> left_drivetrain_talon_;
  ::std::unique_ptr<Talon> right_drivetrain_talon_;
};

class ShooterWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_shooter_talon(::std::unique_ptr<Talon> t) {
    shooter_talon_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2014::control_loops::shooter_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2014::control_loops::shooter_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    shooter_talon_->Set(queue->voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "shooter output too old\n");
    shooter_talon_->Disable();
  }

  ::std::unique_ptr<Talon> shooter_talon_;
};

class ClawWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_top_claw_talon(::std::unique_ptr<Talon> t) {
    top_claw_talon_ = ::std::move(t);
  }

  void set_bottom_claw_talon(::std::unique_ptr<Talon> t) {
    bottom_claw_talon_ = ::std::move(t);
  }

  void set_left_tusk_talon(::std::unique_ptr<Talon> t) {
    left_tusk_talon_ = ::std::move(t);
  }

  void set_right_tusk_talon(::std::unique_ptr<Talon> t) {
    right_tusk_talon_ = ::std::move(t);
  }

  void set_intake1_talon(::std::unique_ptr<Talon> t) {
    intake1_talon_ = ::std::move(t);
  }

  void set_intake2_talon(::std::unique_ptr<Talon> t) {
    intake2_talon_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2014::control_loops::claw_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2014::control_loops::claw_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    intake1_talon_->Set(queue->intake_voltage / 12.0);
    intake2_talon_->Set(queue->intake_voltage / 12.0);
    bottom_claw_talon_->Set(-queue->bottom_claw_voltage / 12.0);
    top_claw_talon_->Set(queue->top_claw_voltage / 12.0);
    left_tusk_talon_->Set(queue->tusk_voltage / 12.0);
    right_tusk_talon_->Set(-queue->tusk_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "claw output too old\n");
    intake1_talon_->Disable();
    intake2_talon_->Disable();
    bottom_claw_talon_->Disable();
    top_claw_talon_->Disable();
    left_tusk_talon_->Disable();
    right_tusk_talon_->Disable();
  }

  ::std::unique_ptr<Talon> top_claw_talon_;
  ::std::unique_ptr<Talon> bottom_claw_talon_;
  ::std::unique_ptr<Talon> left_tusk_talon_;
  ::std::unique_ptr<Talon> right_tusk_talon_;
  ::std::unique_ptr<Talon> intake1_talon_;
  ::std::unique_ptr<Talon> intake2_talon_;
};

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<Encoder> make_encoder(int index) {
    return make_unique<Encoder>(10 + index * 2, 11 + index * 2, false,
                                Encoder::k4X);
  }

  void Run() override {
    ::aos::InitNRT();
    ::aos::SetCurrentThreadName("StartCompetition");

    ::frc971::wpilib::JoystickSender joystick_sender;
    ::std::thread joystick_thread(::std::ref(joystick_sender));

    ::frc971::wpilib::PDPFetcher pdp_fetcher;
    ::std::thread pdp_fetcher_thread(::std::ref(pdp_fetcher));
    SensorReader reader;

    // Create this first to make sure it ends up in one of the lower-numbered
    // FPGA slots so we can use it with DMA.
    auto shooter_encoder_temp = make_encoder(2);

    reader.set_auto_selector_analog(make_unique<AnalogInput>(4));

    reader.set_drivetrain_left_encoder(make_encoder(0));
    reader.set_drivetrain_right_encoder(make_encoder(1));
    reader.set_high_left_drive_hall(make_unique<AnalogInput>(1));
    reader.set_low_left_drive_hall(make_unique<AnalogInput>(0));
    reader.set_high_right_drive_hall(make_unique<AnalogInput>(2));
    reader.set_low_right_drive_hall(make_unique<AnalogInput>(3));

    reader.set_top_claw_encoder(make_encoder(3));
    reader.set_top_claw_front_hall(make_unique<DigitalInput>(4));  // R2
    reader.set_top_claw_calibration_hall(make_unique<DigitalInput>(3));  // R3
    reader.set_top_claw_back_hall(make_unique<DigitalInput>(5));  // R1

    reader.set_bottom_claw_encoder(make_encoder(4));
    reader.set_bottom_claw_front_hall(make_unique<DigitalInput>(1));  // L2
    reader.set_bottom_claw_calibration_hall(make_unique<DigitalInput>(0));  // L3
    reader.set_bottom_claw_back_hall(make_unique<DigitalInput>(2));  // L1

    reader.set_shooter_encoder(::std::move(shooter_encoder_temp));
    reader.set_shooter_proximal(make_unique<DigitalInput>(6));  // S1
    reader.set_shooter_distal(make_unique<DigitalInput>(7));  // S2
    reader.set_shooter_plunger(make_unique<DigitalInput>(8));  // S3
    reader.set_shooter_latch(make_unique<DigitalInput>(9));  // S4

    reader.set_dma(make_unique<DMA>());
    ::std::thread reader_thread(::std::ref(reader));

    ::frc971::wpilib::GyroSender gyro_sender;
    ::std::thread gyro_thread(::std::ref(gyro_sender));

    DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_left_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(5)));
    drivetrain_writer.set_right_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(2)));
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    ::y2014::wpilib::ClawWriter claw_writer;
    claw_writer.set_top_claw_talon(::std::unique_ptr<Talon>(new Talon(1)));
    claw_writer.set_bottom_claw_talon(::std::unique_ptr<Talon>(new Talon(0)));
    claw_writer.set_left_tusk_talon(::std::unique_ptr<Talon>(new Talon(4)));
    claw_writer.set_right_tusk_talon(::std::unique_ptr<Talon>(new Talon(3)));
    claw_writer.set_intake1_talon(::std::unique_ptr<Talon>(new Talon(7)));
    claw_writer.set_intake2_talon(::std::unique_ptr<Talon>(new Talon(8)));
    ::std::thread claw_writer_thread(::std::ref(claw_writer));

    ::y2014::wpilib::ShooterWriter shooter_writer;
    shooter_writer.set_shooter_talon(::std::unique_ptr<Talon>(new Talon(6)));
    ::std::thread shooter_writer_thread(::std::ref(shooter_writer));

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_drivetrain_left(pcm->MakeSolenoid(6));
    solenoid_writer.set_drivetrain_right(pcm->MakeSolenoid(7));
    solenoid_writer.set_shooter_latch(pcm->MakeSolenoid(5));
    solenoid_writer.set_shooter_brake(pcm->MakeSolenoid(4));

    solenoid_writer.set_pressure_switch(make_unique<DigitalInput>(25));
    solenoid_writer.set_compressor_relay(make_unique<Relay>(0));
    ::std::thread solenoid_thread(::std::ref(solenoid_writer));

    // Wait forever. Not much else to do...
    while (true) {
      const int r = select(0, nullptr, nullptr, nullptr, nullptr);
      if (r != 0) {
        PLOG(WARNING, "infinite select failed");
      } else {
        PLOG(WARNING, "infinite select succeeded??\n");
      }
    }

    LOG(ERROR, "Exiting WPILibRobot\n");

    joystick_sender.Quit();
    joystick_thread.join();
    pdp_fetcher.Quit();
    pdp_fetcher_thread.join();
    reader.Quit();
    reader_thread.join();
    gyro_sender.Quit();
    gyro_thread.join();

    drivetrain_writer.Quit();
    drivetrain_writer_thread.join();
    shooter_writer.Quit();
    shooter_writer_thread.join();
    claw_writer.Quit();
    claw_writer_thread.join();
    solenoid_writer.Quit();
    solenoid_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace wpilib
}  // namespace y2014


AOS_ROBOT_CLASS(::y2014::wpilib::WPILibRobot);
