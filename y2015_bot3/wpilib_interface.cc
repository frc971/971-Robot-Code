#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include <chrono>
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
#ifndef WPILIB2015
#include "DigitalGlitchFilter.h"
#endif
#include "DigitalInput.h"
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

#include "frc971/control_loops/control_loops.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/gyro_sender.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging.q.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/wpilib_interface.h"
#include "y2015_bot3/autonomous/auto.q.h"
#include "y2015_bot3/control_loops/drivetrain/drivetrain_base.h"
#include "y2015_bot3/control_loops/elevator/elevator.h"
#include "y2015_bot3/control_loops/elevator/elevator.q.h"
#include "y2015_bot3/control_loops/intake/intake.h"
#include "y2015_bot3/control_loops/intake/intake.q.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::aos::util::SimpleLogInterval;
using ::frc971::control_loops::drivetrain_queue;
using ::y2015_bot3::control_loops::elevator_queue;
using ::y2015_bot3::control_loops::intake_queue;
using ::frc971::wpilib::BufferedPcm;
using ::frc971::wpilib::BufferedSolenoid;
using ::frc971::wpilib::LoopOutputHandler;
using ::frc971::wpilib::JoystickSender;
using ::frc971::wpilib::GyroSender;

namespace y2015_bot3 {
namespace wpilib {

namespace chrono = ::std::chrono;

double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*4x*/) *
         ::y2015_bot3::control_loops::drivetrain::kDrivetrainEncoderRatio *
         (4 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

double drivetrain_velocity_translate(double in) {
  return (1.0 / in) / 256.0 /*cpr*/ *
         ::y2015_bot3::control_loops::drivetrain::kDrivetrainEncoderRatio *
         (4 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

double elevator_translate(int32_t in) {
  return static_cast<double>(in) / (512.0 /*cpr*/ * 4.0 /*4x*/) *
         ::y2015_bot3::control_loops::kElevEncoderRatio * (2 * M_PI /*radians*/) *
         ::y2015_bot3::control_loops::kElevChainReduction *
         ::y2015_bot3::control_loops::kElevGearboxOutputRadianDistance;
}

static const double kMaximumEncoderPulsesPerSecond =
    4650.0 /* free speed RPM */ * 18.0 / 48.0 /* belt reduction */ /
    60.0 /* seconds / minute */ * 512.0 /* CPR */ *
    4.0 /* index pulse = 1/4 cycle */;

// Reads in our inputs. (sensors, voltages, etc.)
class SensorReader {
 public:
  SensorReader() {
    // Set it to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    filter_.SetPeriodNanoSeconds(
        static_cast<int>(1 / 4.0 / kMaximumEncoderPulsesPerSecond * 1e9 + 0.5));
  }

  // Drivetrain setters.
  void set_left_encoder(::std::unique_ptr<Encoder> left_encoder) {
    left_encoder_ = ::std::move(left_encoder);
    left_encoder_->SetMaxPeriod(0.005);
  }

  void set_right_encoder(::std::unique_ptr<Encoder> right_encoder) {
    right_encoder_ = ::std::move(right_encoder);
    right_encoder_->SetMaxPeriod(0.005);
  }

  // Elevator setters.
  void set_elevator_encoder(::std::unique_ptr<Encoder> encoder) {
    filter_.Add(encoder.get());
    elevator_encoder_ = ::std::move(encoder);
  }

  void set_elevator_zeroing_hall_effect(::std::unique_ptr<DigitalInput> hall) {
    zeroing_hall_effect_ = ::std::move(hall);
  }

  void set_elevator_tote_sensor(::std::unique_ptr<AnalogInput> tote_sensor) {
    tote_sensor_ = ::std::move(tote_sensor);
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

    ::aos::time::PhasedLoop phased_loop(chrono::milliseconds(5),
                                        chrono::milliseconds(4));

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

    // Drivetrain
    {
      auto drivetrain_message = drivetrain_queue.position.MakeMessage();
      drivetrain_message->right_encoder =
          -drivetrain_translate(right_encoder_->GetRaw());
      drivetrain_message->left_encoder =
          drivetrain_translate(left_encoder_->GetRaw());
      drivetrain_message->left_speed =
          drivetrain_velocity_translate(left_encoder_->GetPeriod());
      drivetrain_message->right_speed =
          drivetrain_velocity_translate(right_encoder_->GetPeriod());

      drivetrain_message.Send();
    }

    // Elevator
    {
      // Update control loop positions.
      auto elevator_message = elevator_queue.position.MakeMessage();
      elevator_message->encoder =
          elevator_translate(elevator_encoder_->GetRaw());
      elevator_message->bottom_hall_effect = !zeroing_hall_effect_->Get();
      elevator_message->has_tote = tote_sensor_->GetVoltage() > 2.5;

      elevator_message.Send();
    }

    // Intake
    {
      auto intake_message = intake_queue.position.MakeMessage();
      intake_message.Send();
    }
  }

  void Quit() { run_ = false; }

 private:
  int32_t my_pid_;
  DriverStation *ds_;

  ::std::unique_ptr<Encoder> left_encoder_, right_encoder_, elevator_encoder_;
  ::std::unique_ptr<DigitalInput> zeroing_hall_effect_;
  ::std::unique_ptr<AnalogInput> tote_sensor_;

  ::std::atomic<bool> run_{true};
  DigitalGlitchFilter filter_;
};

// Writes out our pneumatic outputs.
class SolenoidWriter {
 public:
  SolenoidWriter(const ::std::unique_ptr< ::frc971::wpilib::BufferedPcm> &pcm)
      : pcm_(pcm),
        elevator_(".y2015_bot3.control_loops.elevator_queue.output"),
        intake_(".y2015_bot3.control_loops.intake_queue.output"),
        can_grabber_control_(".y2015_bot3.autonomous.can_grabber_control") {}

  void set_pressure_switch(::std::unique_ptr<DigitalInput> pressure_switch) {
    pressure_switch_ = ::std::move(pressure_switch);
  }

  void set_compressor_relay(::std::unique_ptr<Relay> compressor_relay) {
    compressor_relay_ = ::std::move(compressor_relay);
  }

  void set_elevator_passive_support(
      ::std::unique_ptr<BufferedSolenoid> elevator_passive_support) {
    elevator_passive_support_ = ::std::move(elevator_passive_support);
  }

  void set_can_grabber(::std::unique_ptr<BufferedSolenoid> can_grabber) {
    can_grabber_ = ::std::move(can_grabber);
  }

  void set_elevator_can_support(
      ::std::unique_ptr<BufferedSolenoid> elevator_can_support) {
    elevator_can_support_ = ::std::move(elevator_can_support);
  }

  void set_intake_claw(::std::unique_ptr<BufferedSolenoid> intake_claw) {
    intake_claw_ = ::std::move(intake_claw);
  }

  void operator()() {
    ::aos::SetCurrentThreadName("Solenoids");
    ::aos::SetCurrentThreadRealtimePriority(27);

    ::aos::time::PhasedLoop phased_loop(chrono::milliseconds(20),
                                        chrono::milliseconds(1));

    while (run_) {
      {
        const int iterations = phased_loop.SleepUntilNext();
        if (iterations != 1) {
          LOG(DEBUG, "Solenoids skipped %d iterations\n", iterations - 1);
        }
      }

      // Can Grabber
      {
        can_grabber_control_.FetchLatest();
        if (can_grabber_control_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *can_grabber_control_);
          can_grabber_->Set(can_grabber_control_->can_grabbers);
        }
      }

      // Elevator
      {
        elevator_.FetchLatest();
        if (elevator_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *elevator_);
          elevator_passive_support_->Set(!elevator_->passive_support);
          elevator_can_support_->Set(!elevator_->can_support);
        }
      }

      // Intake
      {
        intake_.FetchLatest();
        if (intake_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *intake_);
          intake_claw_->Set(intake_->claw_closed);
        }
      }

      // Compressor
      ::aos::joystick_state.FetchLatest();
      {
        ::frc971::wpilib::PneumaticsToLog to_log;
        {
          // Refill if pneumatic pressure goes too low.
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
  const ::std::unique_ptr<BufferedPcm> &pcm_;

  ::std::unique_ptr<BufferedSolenoid> elevator_passive_support_;
  ::std::unique_ptr<BufferedSolenoid> elevator_can_support_;
  ::std::unique_ptr<BufferedSolenoid> intake_claw_;
  ::std::unique_ptr<BufferedSolenoid> can_grabber_;

  ::std::unique_ptr<DigitalInput> pressure_switch_;
  ::std::unique_ptr<Relay> compressor_relay_;

  ::aos::Queue<::y2015_bot3::control_loops::ElevatorQueue::Output> elevator_;
  ::aos::Queue<::y2015_bot3::control_loops::IntakeQueue::Output> intake_;
  ::aos::Queue<::y2015_bot3::autonomous::CanGrabberControl> can_grabber_control_;

  ::std::atomic<bool> run_{true};
};

// Writes out drivetrain voltages.
class DrivetrainWriter : public LoopOutputHandler {
 public:
  void set_left_drivetrain_talon(::std::unique_ptr<Talon> t) {
    left_drivetrain_talon_ = ::std::move(t);
  }

  void set_right_drivetrain_talon(::std::unique_ptr<Talon> t) {
    right_drivetrain_talon_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    drivetrain_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = drivetrain_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    left_drivetrain_talon_->SetSpeed(queue->left_voltage / 12.0);
    right_drivetrain_talon_->SetSpeed(-queue->right_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "drivetrain output too old\n");
    left_drivetrain_talon_->SetDisabled();
    right_drivetrain_talon_->SetDisabled();
  }

  ::std::unique_ptr<Talon> left_drivetrain_talon_;
  ::std::unique_ptr<Talon> right_drivetrain_talon_;
};

// Writes out elevator voltages.
class ElevatorWriter : public LoopOutputHandler {
 public:
  void set_elevator_talon1(::std::unique_ptr<Talon> t) {
    elevator_talon1_ = ::std::move(t);
  }
  void set_elevator_talon2(::std::unique_ptr<Talon> t) {
    elevator_talon2_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2015_bot3::control_loops::elevator_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2015_bot3::control_loops::elevator_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    elevator_talon1_->SetSpeed(queue->elevator / 12.0);
    elevator_talon2_->SetSpeed(-queue->elevator / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Elevator output too old.\n");
    elevator_talon1_->SetDisabled();
    elevator_talon2_->SetDisabled();
  }

  ::std::unique_ptr<Talon> elevator_talon1_;
  ::std::unique_ptr<Talon> elevator_talon2_;
};

// Writes out intake voltages.
class IntakeWriter : public LoopOutputHandler {
 public:
  void set_intake_talon1(::std::unique_ptr<Talon> t) {
    intake_talon1_ = ::std::move(t);
  }
  void set_intake_talon2(::std::unique_ptr<Talon> t) {
    intake_talon2_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2015_bot3::control_loops::intake_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2015_bot3::control_loops::intake_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    intake_talon1_->SetSpeed(queue->intake / 12.0);
    intake_talon2_->SetSpeed(-queue->intake / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Intake output too old.\n");
    intake_talon1_->SetDisabled();
    intake_talon2_->SetDisabled();
  }

  ::std::unique_ptr<Talon> intake_talon1_;
  ::std::unique_ptr<Talon> intake_talon2_;
};

// Writes out can grabber voltages.
class CanGrabberWriter : public LoopOutputHandler {
 public:
  CanGrabberWriter() : LoopOutputHandler(chrono::milliseconds(50)) {}

  void set_can_grabber_talon1(::std::unique_ptr<Talon> t) {
    can_grabber_talon1_ = ::std::move(t);
  }

  void set_can_grabber_talon2(::std::unique_ptr<Talon> t) {
    can_grabber_talon2_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2015_bot3::autonomous::can_grabber_control.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2015_bot3::autonomous::can_grabber_control;
    LOG_STRUCT(DEBUG, "will output", *queue);
    can_grabber_talon1_->SetSpeed(queue->can_grabber_voltage / 12.0);
    can_grabber_talon2_->SetSpeed(-queue->can_grabber_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Can grabber output too old\n");
    can_grabber_talon1_->SetDisabled();
    can_grabber_talon2_->SetDisabled();
  }

  ::std::unique_ptr<Talon> can_grabber_talon1_, can_grabber_talon2_;
};

// TODO(brian): Replace this with ::std::make_unique once all our toolchains
// have support.
template <class T, class... U>
std::unique_ptr<T> make_unique(U &&... u) {
  return std::unique_ptr<T>(new T(std::forward<U>(u)...));
}

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<Encoder> encoder(int index) {
    return make_unique<Encoder>(10 + index * 2, 11 + index * 2, false,
                                Encoder::k4X);
  }
  virtual void Run() {
    ::aos::InitNRT();
    ::aos::SetCurrentThreadName("StartCompetition");

    JoystickSender joystick_sender;
    ::std::thread joystick_thread(::std::ref(joystick_sender));

    ::frc971::wpilib::PDPFetcher pdp_fetcher;
    ::std::thread pdp_fetcher_thread(::std::ref(pdp_fetcher));
    SensorReader reader;

    reader.set_elevator_encoder(encoder(6));
    reader.set_elevator_zeroing_hall_effect(make_unique<DigitalInput>(6));

    reader.set_left_encoder(encoder(0));
    reader.set_right_encoder(encoder(1));
    reader.set_elevator_tote_sensor(make_unique<AnalogInput>(0));

    ::std::thread reader_thread(::std::ref(reader));
    GyroSender gyro_sender;
    ::std::thread gyro_thread(::std::ref(gyro_sender));

    DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_left_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(0)));
    drivetrain_writer.set_right_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(7)));
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    ElevatorWriter elevator_writer;
    elevator_writer.set_elevator_talon1(::std::unique_ptr<Talon>(new Talon(1)));
    elevator_writer.set_elevator_talon2(::std::unique_ptr<Talon>(new Talon(6)));
    ::std::thread elevator_writer_thread(::std::ref(elevator_writer));

    IntakeWriter intake_writer;
    intake_writer.set_intake_talon1(::std::unique_ptr<Talon>(new Talon(2)));
    intake_writer.set_intake_talon2(::std::unique_ptr<Talon>(new Talon(5)));
    ::std::thread intake_writer_thread(::std::ref(intake_writer));

    CanGrabberWriter can_grabber_writer;
    can_grabber_writer.set_can_grabber_talon1(
        ::std::unique_ptr<Talon>(new Talon(3)));
    can_grabber_writer.set_can_grabber_talon2(
        ::std::unique_ptr<Talon>(new Talon(4)));
    ::std::thread can_grabber_writer_thread(::std::ref(can_grabber_writer));

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_pressure_switch(make_unique<DigitalInput>(9));
    solenoid_writer.set_compressor_relay(make_unique<Relay>(0));
    solenoid_writer.set_elevator_passive_support(pcm->MakeSolenoid(0));
    solenoid_writer.set_elevator_can_support(pcm->MakeSolenoid(1));
    solenoid_writer.set_intake_claw(pcm->MakeSolenoid(2));
    solenoid_writer.set_can_grabber(pcm->MakeSolenoid(3));
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

    elevator_writer.Quit();
    elevator_writer_thread.join();

    intake_writer.Quit();
    intake_writer_thread.join();

    can_grabber_writer.Quit();
    can_grabber_writer_thread.join();

    solenoid_writer.Quit();
    solenoid_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace wpilib
}  // namespace y2015_bot3

AOS_ROBOT_CLASS(::y2015_bot3::wpilib::WPILibRobot);
