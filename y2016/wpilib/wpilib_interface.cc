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
#include "aos/common/commonmath.h"

#include "frc971/control_loops/control_loops.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2016/control_loops/shooter/shooter.q.h"
#include "y2016/constants.h"
#include "y2016/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2016/control_loops/shooter/shooter.q.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"

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

using ::frc971::control_loops::drivetrain_queue;
using ::y2016::control_loops::shooter::shooter_queue;
using ::y2016::control_loops::superstructure_queue;

namespace y2016 {
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

// Translates for the sensor values to convert raw index pulses into something
// with proper units.

// TODO(comran): Template these methods since there is a lot of repetition here.
double hall_translate(double in) {
  // Turn voltage from our 3-state halls into a ratio that the loop can use.
  return in / 5.0;
}

double drivetrain_translate(int32_t in) {
  return -static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*4x*/) *
         constants::Values::kDrivetrainEncoderRatio *
         control_loops::drivetrain::kWheelRadius * 2.0 * M_PI;
}

double drivetrain_velocity_translate(double in) {
  return (1.0 / in) / 256.0 /*cpr*/ *
         constants::Values::kDrivetrainEncoderRatio *
         control_loops::drivetrain::kWheelRadius * 2.0 * M_PI;
}

double shooter_translate(int32_t in) {
  return -static_cast<double>(in) / (128.0 /*cpr*/ * 4.0 /*4x*/) *
         constants::Values::kShooterEncoderRatio * (2 * M_PI /*radians*/);
}

double intake_translate(int32_t in) {
  return static_cast<double>(in) / (512.0 /*cpr*/ * 4.0 /*4x*/) *
         constants::Values::kIntakeEncoderRatio * (2 * M_PI /*radians*/);
}

double shoulder_translate(int32_t in) {
  return -static_cast<double>(in) / (512.0 /*cpr*/ * 4.0 /*4x*/) *
         constants::Values::kShoulderEncoderRatio * (2 * M_PI /*radians*/);
}

double wrist_translate(int32_t in) {
  return -static_cast<double>(in) / (512.0 /*cpr*/ * 4.0 /*4x*/) *
         constants::Values::kWristEncoderRatio * (2 * M_PI /*radians*/);
}

double intake_pot_translate(double voltage) {
  return voltage * constants::Values::kIntakePotRatio *
         (10.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

double shoulder_pot_translate(double voltage) {
  return voltage * constants::Values::kShoulderPotRatio *
         (3.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

double wrist_pot_translate(double voltage) {
  return voltage * constants::Values::kWristPotRatio *
         (3.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

constexpr double kMaxDrivetrainEncoderPulsesPerSecond =
    5600.0 /* CIM free speed RPM */ * 14.0 / 48.0 /* 1st reduction */ * 28.0 /
    50.0 /* 2nd reduction (high gear) */ * 30.0 / 44.0 /* encoder gears */ /
    60.0 /* seconds per minute */ * 256.0 /* CPR */ * 4 /* edges per cycle */;

constexpr double kMaxShooterEncoderPulsesPerSecond =
    18700.0 /* 775pro free speed RPM */ * 12.0 /
    18.0 /* motor to encoder reduction */ / 60.0 /* seconds per minute */ *
    128.0 /* CPR */ * 4 /* edges per cycle */;

double kMaxDrivetrainShooterEncoderPulsesPerSecond = ::std::max(
    kMaxDrivetrainEncoderPulsesPerSecond, kMaxShooterEncoderPulsesPerSecond);

constexpr double kMaxSuperstructureEncoderPulsesPerSecond =
    18700.0 /* 775pro free speed RPM */ * 12.0 /
    56.0 /* motor to encoder reduction */ / 60.0 /* seconds per minute */ *
    512.0 /* CPR */ * 4 /* index pulse every quarter cycle */;

// Class to send position messages with sensor readings to our loops.
class SensorReader {
 public:
  SensorReader() {
    // Set it to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    drivetrain_shooter_encoder_filter_.SetPeriodNanoSeconds(
        static_cast<int>(1 / 4.0 /* built-in tolerance */ /
                             kMaxDrivetrainShooterEncoderPulsesPerSecond * 1e9 +
                         0.5));
    superstructure_encoder_filter_.SetPeriodNanoSeconds(
        static_cast<int>(1 / 4.0 /* built-in tolerance */ /
                             kMaxSuperstructureEncoderPulsesPerSecond * 1e9 +
                         0.5));
    hall_filter_.SetPeriodNanoSeconds(100000);
  }

  // Drivetrain setters.
  void set_drivetrain_left_encoder(::std::unique_ptr<Encoder> encoder) {
    drivetrain_shooter_encoder_filter_.Add(encoder.get());
    drivetrain_left_encoder_ = ::std::move(encoder);
  }

  void set_drivetrain_right_encoder(::std::unique_ptr<Encoder> encoder) {
    drivetrain_shooter_encoder_filter_.Add(encoder.get());
    drivetrain_right_encoder_ = ::std::move(encoder);
  }

  void set_drivetrain_left_hall(::std::unique_ptr<AnalogInput> analog) {
    drivetrain_left_hall_ = ::std::move(analog);
  }

  void set_drivetrain_right_hall(::std::unique_ptr<AnalogInput> analog) {
    drivetrain_right_hall_ = ::std::move(analog);
  }

  // Shooter setters.
  void set_shooter_left_encoder(::std::unique_ptr<Encoder> encoder) {
    drivetrain_shooter_encoder_filter_.Add(encoder.get());
    shooter_left_encoder_ = ::std::move(encoder);
  }

  void set_shooter_right_encoder(::std::unique_ptr<Encoder> encoder) {
    drivetrain_shooter_encoder_filter_.Add(encoder.get());
    shooter_right_encoder_ = ::std::move(encoder);
  }

  // Intake setters.
  void set_intake_encoder(::std::unique_ptr<Encoder> encoder) {
    superstructure_encoder_filter_.Add(encoder.get());
    intake_encoder_.set_encoder(::std::move(encoder));
  }

  void set_intake_potentiometer(::std::unique_ptr<AnalogInput> potentiometer) {
    intake_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_intake_index(::std::unique_ptr<DigitalInput> index) {
    superstructure_encoder_filter_.Add(index.get());
    intake_encoder_.set_index(::std::move(index));
  }

  // Shoulder setters.
  void set_shoulder_encoder(::std::unique_ptr<Encoder> encoder) {
    superstructure_encoder_filter_.Add(encoder.get());
    shoulder_encoder_.set_encoder(::std::move(encoder));
  }

  void set_shoulder_potentiometer(
      ::std::unique_ptr<AnalogInput> potentiometer) {
    shoulder_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_shoulder_index(::std::unique_ptr<DigitalInput> index) {
    superstructure_encoder_filter_.Add(index.get());
    shoulder_encoder_.set_index(::std::move(index));
  }

  // Wrist setters.
  void set_wrist_encoder(::std::unique_ptr<Encoder> encoder) {
    superstructure_encoder_filter_.Add(encoder.get());
    wrist_encoder_.set_encoder(::std::move(encoder));
  }

  void set_wrist_potentiometer(::std::unique_ptr<AnalogInput> potentiometer) {
    wrist_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_wrist_index(::std::unique_ptr<DigitalInput> index) {
    superstructure_encoder_filter_.Add(index.get());
    wrist_encoder_.set_index(::std::move(index));
  }

  // All of the DMA-related set_* calls must be made before this, and it doesn't
  // hurt to do all of them.

  // TODO(comran): Add 2016 things down below for dma synchronization.
  void set_dma(::std::unique_ptr<DMA> dma) {
    dma_synchronizer_.reset(
        new ::frc971::wpilib::DMASynchronizer(::std::move(dma)));
    dma_synchronizer_->Add(&intake_encoder_);
    dma_synchronizer_->Add(&shoulder_encoder_);
    dma_synchronizer_->Add(&wrist_encoder_);
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
  }

  void RunIteration() {
    ::frc971::wpilib::SendRobotState(my_pid_, ds_);

    const auto &values = constants::GetValues();

    {
      auto drivetrain_message = drivetrain_queue.position.MakeMessage();
      drivetrain_message->right_encoder =
          drivetrain_translate(-drivetrain_right_encoder_->GetRaw());
      drivetrain_message->left_encoder =
          -drivetrain_translate(drivetrain_left_encoder_->GetRaw());
      drivetrain_message->left_speed =
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod());
      drivetrain_message->right_speed =
          drivetrain_velocity_translate(drivetrain_right_encoder_->GetPeriod());

      drivetrain_message->left_shifter_position =
          hall_translate(drivetrain_left_hall_->GetVoltage());
      drivetrain_message->right_shifter_position =
          hall_translate(drivetrain_right_hall_->GetVoltage());

      drivetrain_message.Send();
    }

    dma_synchronizer_->RunIteration();

    {
      auto shooter_message = shooter_queue.position.MakeMessage();
      shooter_message->theta_left =
          shooter_translate(-shooter_left_encoder_->GetRaw());
      shooter_message->theta_right =
          shooter_translate(shooter_right_encoder_->GetRaw());
      shooter_message.Send();
    }

    {
      auto superstructure_message = superstructure_queue.position.MakeMessage();
      CopyPotAndIndexPosition(intake_encoder_, &superstructure_message->intake,
                              intake_translate, intake_pot_translate, false,
                              values.intake.pot_offset);
      CopyPotAndIndexPosition(shoulder_encoder_,
                              &superstructure_message->shoulder,
                              shoulder_translate, shoulder_pot_translate, false,
                              values.shoulder.pot_offset);
      CopyPotAndIndexPosition(wrist_encoder_, &superstructure_message->wrist,
                              wrist_translate, wrist_pot_translate, true,
                              values.wrist.pot_offset);

      superstructure_message.Send();
    }
  }

  void Quit() { run_ = false; }

 private:
  void CopyPotAndIndexPosition(
      const ::frc971::wpilib::DMAEncoderAndPotentiometer &encoder,
      ::frc971::PotAndIndexPosition *position,
      ::std::function<double(int32_t)> encoder_translate,
      ::std::function<double(double)> potentiometer_translate, bool reverse,
      double pot_offset) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->encoder =
        multiplier * encoder_translate(encoder.polled_encoder_value());
    position->pot = multiplier * potentiometer_translate(
                                     encoder.polled_potentiometer_voltage()) +
                    pot_offset;
    position->latched_encoder =
        multiplier * encoder_translate(encoder.last_encoder_value());
    position->latched_pot =
        multiplier *
            potentiometer_translate(encoder.last_potentiometer_voltage()) +
        pot_offset;
    position->index_pulses = encoder.index_posedge_count();
  }

  int32_t my_pid_;
  DriverStation *ds_;

  ::std::unique_ptr<::frc971::wpilib::DMASynchronizer> dma_synchronizer_;

  ::std::unique_ptr<Encoder> drivetrain_left_encoder_,
      drivetrain_right_encoder_;
  ::std::unique_ptr<AnalogInput> drivetrain_left_hall_, drivetrain_right_hall_;

  ::std::unique_ptr<Encoder> shooter_left_encoder_, shooter_right_encoder_;
  ::frc971::wpilib::DMAEncoderAndPotentiometer intake_encoder_,
      shoulder_encoder_, wrist_encoder_;

  ::std::atomic<bool> run_{true};
  DigitalGlitchFilter drivetrain_shooter_encoder_filter_, hall_filter_,
      superstructure_encoder_filter_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm)
      : pcm_(pcm),
        drivetrain_(".frc971.control_loops.drivetrain_queue.output"),
        shooter_(".y2016.control_loops.shooter.shooter_queue.output") {}

  void set_compressor(::std::unique_ptr<Compressor> compressor) {
    compressor_ = ::std::move(compressor);
  }

  void set_drivetrain_left(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    drivetrain_left_ = ::std::move(s);
  }

  void set_drivetrain_right(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    drivetrain_right_ = ::std::move(s);
  }

  void set_shooter_clamp(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    shooter_clamp_ = ::std::move(s);
  }

  void set_shooter_pusher(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    shooter_pusher_ = ::std::move(s);
  }

  void operator()() {
    compressor_->Start();
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
        drivetrain_.FetchLatest();
        if (drivetrain_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *drivetrain_);
          drivetrain_left_->Set(!drivetrain_->left_high);
          drivetrain_right_->Set(!drivetrain_->right_high);
        }
      }

      {
        shooter_.FetchLatest();
        if (shooter_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *shooter_);
          shooter_clamp_->Set(shooter_->clamp_open);
          shooter_pusher_->Set(shooter_->push_to_shooter);
        }
      }

      {
        ::frc971::wpilib::PneumaticsToLog to_log;
        {
          to_log.compressor_on = compressor_->Enabled();
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

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> drivetrain_left_,
      drivetrain_right_, shooter_clamp_, shooter_pusher_;
  ::std::unique_ptr<Compressor> compressor_;

  ::aos::Queue<::frc971::control_loops::DrivetrainQueue::Output> drivetrain_;
  ::aos::Queue<::y2016::control_loops::shooter::ShooterQueue::Output> shooter_;

  ::std::atomic<bool> run_{true};
};

class DrivetrainWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_drivetrain_left_talon(::std::unique_ptr<Talon> t) {
    drivetrain_left_talon_ = ::std::move(t);
  }

  void set_drivetrain_right_talon(::std::unique_ptr<Talon> t) {
    drivetrain_right_talon_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::frc971::control_loops::drivetrain_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::frc971::control_loops::drivetrain_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    drivetrain_left_talon_->Set(queue->left_voltage / 12.0);
    drivetrain_right_talon_->Set(-queue->right_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "drivetrain output too old\n");
    drivetrain_left_talon_->Disable();
    drivetrain_right_talon_->Disable();
  }

  ::std::unique_ptr<Talon> drivetrain_left_talon_, drivetrain_right_talon_;
};

class ShooterWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_shooter_left_talon(::std::unique_ptr<Talon> t) {
    shooter_left_talon_ = ::std::move(t);
  }

  void set_shooter_right_talon(::std::unique_ptr<Talon> t) {
    shooter_right_talon_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2016::control_loops::shooter::shooter_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2016::control_loops::shooter::shooter_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);

    shooter_left_talon_->Set(queue->voltage_left / 12.0);
    shooter_right_talon_->Set(-queue->voltage_right / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Shooter output too old.\n");
    shooter_left_talon_->Disable();
    shooter_right_talon_->Disable();
  }

  ::std::unique_ptr<Talon> shooter_left_talon_, shooter_right_talon_;
};

class SuperstructureWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_intake_talon(::std::unique_ptr<Talon> t) {
    intake_talon_ = ::std::move(t);
  }

  void set_shoulder_talon(::std::unique_ptr<Talon> t) {
    shoulder_talon_ = ::std::move(t);
  }

  void set_wrist_talon(::std::unique_ptr<Talon> t) {
    wrist_talon_ = ::std::move(t);
  }

  void set_top_rollers_talon(::std::unique_ptr<Talon> t) {
    top_rollers_talon_ = ::std::move(t);
  }

  void set_bottom_rollers_talon(::std::unique_ptr<Talon> t) {
    bottom_rollers_talon_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2016::control_loops::superstructure_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2016::control_loops::superstructure_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    intake_talon_->Set(queue->voltage_intake / 12.0);
    shoulder_talon_->Set(::aos::Clip(-queue->voltage_shoulder, -6.0, 6.0) / 12.0);
    wrist_talon_->Set(::aos::Clip(queue->voltage_wrist, -6.0, 6.0) / 12.0);
    top_rollers_talon_->Set(-queue->voltage_top_rollers / 12.0);
    bottom_rollers_talon_->Set(-queue->voltage_bottom_rollers / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Superstructure output too old.\n");
    intake_talon_->Disable();
    shoulder_talon_->Disable();
    wrist_talon_->Disable();
  }

  ::std::unique_ptr<Talon> intake_talon_, shoulder_talon_, wrist_talon_,
      top_rollers_talon_, bottom_rollers_talon_;
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

    // TODO(constants): Update these input numbers.
    reader.set_drivetrain_left_encoder(make_encoder(5));
    reader.set_drivetrain_right_encoder(make_encoder(6));
    reader.set_drivetrain_left_hall(make_unique<AnalogInput>(5));
    reader.set_drivetrain_right_hall(make_unique<AnalogInput>(6));

    reader.set_shooter_left_encoder(make_encoder(3));
    reader.set_shooter_right_encoder(make_encoder(4));

    reader.set_intake_encoder(make_encoder(0));
    reader.set_intake_index(make_unique<DigitalInput>(0));
    reader.set_intake_potentiometer(make_unique<AnalogInput>(0));

    reader.set_shoulder_encoder(make_encoder(2));
    reader.set_shoulder_index(make_unique<DigitalInput>(2));
    reader.set_shoulder_potentiometer(make_unique<AnalogInput>(2));

    reader.set_wrist_encoder(make_encoder(1));
    reader.set_wrist_index(make_unique<DigitalInput>(1));
    reader.set_wrist_potentiometer(make_unique<AnalogInput>(1));

    reader.set_dma(make_unique<DMA>());
    ::std::thread reader_thread(::std::ref(reader));

    ::frc971::wpilib::GyroSender gyro_sender;
    ::std::thread gyro_thread(::std::ref(gyro_sender));

    DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_drivetrain_left_talon(
        ::std::unique_ptr<Talon>(new Talon(5)));
    drivetrain_writer.set_drivetrain_right_talon(
        ::std::unique_ptr<Talon>(new Talon(4)));
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    ShooterWriter shooter_writer;
    shooter_writer.set_shooter_left_talon(
        ::std::unique_ptr<Talon>(new Talon(9)));
    shooter_writer.set_shooter_right_talon(
        ::std::unique_ptr<Talon>(new Talon(8)));
    ::std::thread shooter_writer_thread(::std::ref(shooter_writer));

    SuperstructureWriter superstructure_writer;
    superstructure_writer.set_intake_talon(
        ::std::unique_ptr<Talon>(new Talon(3)));
    superstructure_writer.set_shoulder_talon(
        ::std::unique_ptr<Talon>(new Talon(6)));
    superstructure_writer.set_wrist_talon(
        ::std::unique_ptr<Talon>(new Talon(2)));
    superstructure_writer.set_top_rollers_talon(
        ::std::unique_ptr<Talon>(new Talon(1)));
    superstructure_writer.set_bottom_rollers_talon(
        ::std::unique_ptr<Talon>(new Talon(0)));
    ::std::thread superstructure_writer_thread(
        ::std::ref(superstructure_writer));

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_drivetrain_left(pcm->MakeSolenoid(1));
    solenoid_writer.set_drivetrain_right(pcm->MakeSolenoid(0));
    solenoid_writer.set_shooter_clamp(pcm->MakeSolenoid(4));
    solenoid_writer.set_shooter_pusher(pcm->MakeSolenoid(5));

    solenoid_writer.set_compressor(make_unique<Compressor>());

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
    superstructure_writer.Quit();
    superstructure_writer_thread.join();
    solenoid_writer.Quit();
    solenoid_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace wpilib
}  // namespace y2016

AOS_ROBOT_CLASS(::y2016::wpilib::WPILibRobot);
