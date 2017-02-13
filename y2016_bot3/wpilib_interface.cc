#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include <thread>
#include <mutex>
#include <functional>
#include <array>

#include "Encoder.h"
#include "Talon.h"
#include "Relay.h"
#include "DriverStation.h"
#include "AnalogInput.h"
#include "Compressor.h"
#include "frc971/wpilib/wpilib_robot_base.h"
#include "DigitalGlitchFilter.h"
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
#include "y2016_bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2016_bot3/control_loops/intake/intake.q.h"
#include "y2016_bot3/queues/ball_detector.q.h"
#include "y2016_bot3/actors/autonomous_action.q.h"

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
#include "frc971/wpilib/dma.h"

#include "y2016_bot3/control_loops/intake/intake.h"
#include "y2016_bot3/control_loops/drivetrain/drivetrain_base.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain_queue;
using ::y2016_bot3::control_loops::intake_queue;

namespace y2016_bot3 {
namespace constants {
IntakeZero intake_zero;
}
namespace wpilib {
namespace {
constexpr double kMaxBringupPower = 12.0;
}  // namespace

// TODO(Brian): Fix the interpretation of the result of GetRaw here and in the
// DMA stuff and then removing the * 2.0 in *_translate.
// The low bit is direction.

// TODO(brian): Replace this with ::std::make_unique once all our toolchains
// have support.
template <class T, class... U>
std::unique_ptr<T> make_unique(U &&... u) {
  return std::unique_ptr<T>(new T(std::forward<U>(u)...));
}

// TODO(Campbell): Update values
// Translates for the sensor values to convert raw index pulses into something
// with proper units.

double drivetrain_translate(int32_t in) {
  return -static_cast<double>(in) / (512.0 /*cpr*/ * 4.0 /*4x*/) *
         ::y2016_bot3::constants::kDrivetrainEncoderRatio *
         control_loops::drivetrain::kWheelRadius * 2.0 * M_PI;
}

double drivetrain_velocity_translate(double in) {
  return (1.0 / in) / 512.0 /*cpr*/ *
         ::y2016_bot3::constants::kDrivetrainEncoderRatio *
         control_loops::drivetrain::kWheelRadius * 2.0 * M_PI;
}

double intake_translate(int32_t in) {
  return static_cast<double>(in) / (512.0 /*cpr*/ * 4.0 /*4x*/) *
         ::y2016_bot3::constants::kIntakeEncoderRatio * (2 * M_PI /*radians*/);
}

double intake_pot_translate(double voltage) {
  return voltage * ::y2016_bot3::constants::kIntakePotRatio *
         (5.0 /*turns*/ / 5.0 /*volts*/) * (2 * M_PI /*radians*/);
}

constexpr double kMaxDrivetrainEncoderPulsesPerSecond =
    5600.0 /* CIM free speed RPM */ * 14.0 / 48.0 /* 1st reduction */ * 28.0 /
    50.0 /* 2nd reduction (high gear) */ * 30.0 / 44.0 /* encoder gears */ /
    60.0 /* seconds per minute */ * 256.0 /* CPR */ * 4 /* edges per cycle */;

// Class to send position messages with sensor readings to our loops.
class SensorReader {
 public:
  SensorReader() {
    // Set it to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    drivetrain_encoder_filter_.SetPeriodNanoSeconds(
        static_cast<int>(1 / 4.0 /* built-in tolerance */ /
                             kMaxDrivetrainEncoderPulsesPerSecond * 1e9 +
                         0.5));
  }

  // Drivetrain setters.
  void set_drivetrain_left_encoder(::std::unique_ptr<Encoder> encoder) {
    drivetrain_encoder_filter_.Add(encoder.get());
    drivetrain_left_encoder_ = ::std::move(encoder);
  }

  void set_drivetrain_right_encoder(::std::unique_ptr<Encoder> encoder) {
    drivetrain_encoder_filter_.Add(encoder.get());
    drivetrain_right_encoder_ = ::std::move(encoder);
  }

  // Intake setters.
  void set_intake_encoder(::std::unique_ptr<Encoder> encoder) {
    intake_encoder_filter_.Add(encoder.get());
    intake_encoder_.set_encoder(::std::move(encoder));
  }

  void set_intake_potentiometer(::std::unique_ptr<AnalogInput> potentiometer) {
    intake_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_intake_index(::std::unique_ptr<DigitalInput> index) {
    intake_encoder_filter_.Add(index.get());
    intake_encoder_.set_index(::std::move(index));
  }

  // Ball detector setter.
  void set_ball_detector(::std::unique_ptr<AnalogInput> analog) {
    ball_detector_ = ::std::move(analog);
  }

  // All of the DMA-related set_* calls must be made before this, and it doesn't
  // hurt to do all of them.

  void set_dma(::std::unique_ptr<DMA> dma) {
    dma_synchronizer_.reset(
        new ::frc971::wpilib::DMASynchronizer(::std::move(dma)));
    dma_synchronizer_->Add(&intake_encoder_);
  }

  void operator()() {
    ::aos::SetCurrentThreadName("SensorReader");

    my_pid_ = getpid();
    ds_ = &DriverStation::GetInstance();

    dma_synchronizer_->Start();

    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                        ::std::chrono::milliseconds(0));

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

    const auto intake_pot_offset =
        y2016_bot3::constants::intake_zero.pot_offset;

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

      drivetrain_message.Send();
    }

    dma_synchronizer_->RunIteration();

    {
      auto intake_message = intake_queue.position.MakeMessage();
      CopyPotAndIndexPosition(intake_encoder_, &intake_message->intake,
                              intake_translate, intake_pot_translate, true,
                              intake_pot_offset);

      intake_message.Send();
    }

    {
      auto ball_detector_message =
          ::y2016_bot3::sensors::ball_detector.MakeMessage();
      ball_detector_message->voltage = ball_detector_->GetVoltage();
      LOG_STRUCT(DEBUG, "ball detector", *ball_detector_message);
      ball_detector_message.Send();
    }

    {
      auto auto_mode_message = ::y2016_bot3::actors::auto_mode.MakeMessage();
      auto_mode_message->mode = 0;
      LOG_STRUCT(DEBUG, "auto mode", *auto_mode_message);
      auto_mode_message.Send();
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

  ::frc971::wpilib::DMAEncoderAndPotentiometer intake_encoder_;
  ::std::unique_ptr<AnalogInput> ball_detector_;

  ::std::atomic<bool> run_{true};
  DigitalGlitchFilter drivetrain_encoder_filter_, intake_encoder_filter_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm)
      : pcm_(pcm),
        drivetrain_(".frc971.control_loops.drivetrain_queue.output"),
        intake_(".y2016_bot3.control_loops.intake_queue.output") {}

  void set_compressor(::std::unique_ptr<Compressor> compressor) {
    compressor_ = ::std::move(compressor);
  }

  void set_traverse(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    traverse_ = ::std::move(s);
  }

  void operator()() {
    compressor_->Start();
    ::aos::SetCurrentThreadName("Solenoids");
    ::aos::SetCurrentThreadRealtimePriority(27);

    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(20),
                                        ::std::chrono::milliseconds(1));

    while (run_) {
      {
        const int iterations = phased_loop.SleepUntilNext();
        if (iterations != 1) {
          LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
        }
      }

      {
        intake_.FetchLatest();
        if (intake_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *intake_);
          traverse_->Set(intake_->traverse_down);
        }
      }

      {
        ::frc971::wpilib::PneumaticsToLog to_log;
        { to_log.compressor_on = compressor_->Enabled(); }

        pcm_->Flush();
        to_log.read_solenoids = pcm_->GetAll();
        LOG_STRUCT(DEBUG, "pneumatics info", to_log);
      }
    }
  }

  void Quit() { run_ = false; }

 private:
  const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm_;

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> traverse_;
  ::std::unique_ptr<Compressor> compressor_;

  ::aos::Queue<::frc971::control_loops::DrivetrainQueue::Output> drivetrain_;
  ::aos::Queue<::y2016_bot3::control_loops::IntakeQueue::Output> intake_;

  ::std::atomic<bool> run_{true};
};

class DrivetrainWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_drivetrain_left_talon(::std::unique_ptr<Talon> t0,
                                 ::std::unique_ptr<Talon> t1) {
    drivetrain_left_talon_0_ = ::std::move(t0);
    drivetrain_left_talon_1_ = ::std::move(t1);
  }

  void set_drivetrain_right_talon(::std::unique_ptr<Talon> t0,
                                  ::std::unique_ptr<Talon> t1) {
    drivetrain_right_talon_0_ = ::std::move(t0);
    drivetrain_right_talon_1_ = ::std::move(t1);
  }

 private:
  virtual void Read() override {
    ::frc971::control_loops::drivetrain_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::frc971::control_loops::drivetrain_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    drivetrain_left_talon_0_->Set(queue->left_voltage / 12.0);
    drivetrain_left_talon_1_->Set(queue->left_voltage / 12.0);
    drivetrain_right_talon_0_->Set(-queue->right_voltage / 12.0);
    drivetrain_right_talon_1_->Set(-queue->right_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "drivetrain output too old\n");
    drivetrain_left_talon_0_->Disable();
    drivetrain_right_talon_0_->Disable();
    drivetrain_left_talon_1_->Disable();
    drivetrain_right_talon_1_->Disable();
  }

  ::std::unique_ptr<Talon> drivetrain_left_talon_0_, drivetrain_right_talon_0_,
      drivetrain_right_talon_1_, drivetrain_left_talon_1_;
};

class IntakeWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_intake_talon(::std::unique_ptr<Talon> t) {
    intake_talon_ = ::std::move(t);
  }

  void set_intake_rollers_talon(::std::unique_ptr<Talon> t) {
    intake_rollers_talon_ = ::std::move(t);
  }

  void set_top_rollers_talon(::std::unique_ptr<Talon> t) {
    top_rollers_talon_ = ::std::move(t);
  }

  void set_bottom_rollers_talon(::std::unique_ptr<Talon> t) {
    bottom_rollers_talon_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2016_bot3::control_loops::intake_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2016_bot3::control_loops::intake_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    intake_talon_->Set(::aos::Clip(queue->voltage_intake, -kMaxBringupPower,
                                   kMaxBringupPower) /
                       12.0);
    top_rollers_talon_->Set(-queue->voltage_top_rollers / 12.0);
    intake_rollers_talon_->Set(-queue->voltage_intake_rollers / 12.0);
    bottom_rollers_talon_->Set(-queue->voltage_bottom_rollers / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Intake output too old.\n");
    intake_talon_->Disable();
  }

  ::std::unique_ptr<Talon> intake_talon_, top_rollers_talon_,
      bottom_rollers_talon_, intake_rollers_talon_;
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

    reader.set_drivetrain_left_encoder(make_encoder(0));
    reader.set_drivetrain_right_encoder(make_encoder(1));

    reader.set_intake_encoder(make_encoder(2));
    reader.set_intake_index(make_unique<DigitalInput>(0));
    reader.set_intake_potentiometer(make_unique<AnalogInput>(4));

    reader.set_ball_detector(make_unique<AnalogInput>(5));

    reader.set_dma(make_unique<DMA>());
    ::std::thread reader_thread(::std::ref(reader));

    ::frc971::wpilib::GyroSender gyro_sender;
    ::std::thread gyro_thread(::std::ref(gyro_sender));

    DrivetrainWriter drivetrain_writer;
    // 2 and 3 are right. 0 and 1 are left
    drivetrain_writer.set_drivetrain_left_talon(
        ::std::unique_ptr<Talon>(new Talon(0)),
        ::std::unique_ptr<Talon>(new Talon(1)));
    drivetrain_writer.set_drivetrain_right_talon(
        ::std::unique_ptr<Talon>(new Talon(2)),
        ::std::unique_ptr<Talon>(new Talon(3)));
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    IntakeWriter intake_writer;
    intake_writer.set_intake_talon(::std::unique_ptr<Talon>(new Talon(7)));
    intake_writer.set_top_rollers_talon(::std::unique_ptr<Talon>(new Talon(6)));
    intake_writer.set_intake_rollers_talon(
        ::std::unique_ptr<Talon>(new Talon(5)));
    intake_writer.set_bottom_rollers_talon(
        ::std::unique_ptr<Talon>(new Talon(4)));
    ::std::thread intake_writer_thread(::std::ref(intake_writer));

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_traverse(pcm->MakeSolenoid(3));

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
    intake_writer.Quit();
    intake_writer_thread.join();
    solenoid_writer.Quit();
    solenoid_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace wpilib
}  // namespace y2016_bot3

AOS_ROBOT_CLASS(::y2016_bot3::wpilib::WPILibRobot);
