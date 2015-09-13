#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include <thread>
#include <mutex>
#include <functional>

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
#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "bot3/control_loops/elevator/elevator.q.h"
#include "bot3/control_loops/intake/intake.q.h"

#include "frc971/wpilib/hall_effect.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/gyro_sender.h"
#include "frc971/wpilib/logging.q.h"
#include "bot3/control_loops/drivetrain/drivetrain.h"
#include "bot3/control_loops/elevator/elevator.h"
#include "bot3/control_loops/intake/intake.h"

#include "Encoder.h"
#include "Talon.h"
#include "DriverStation.h"
#include "AnalogInput.h"
#include "Compressor.h"
#include "Relay.h"
#include "RobotBase.h"
#include "dma.h"
#include "ControllerPower.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::aos::util::SimpleLogInterval;
using ::bot3::control_loops::drivetrain_queue;
using ::bot3::control_loops::elevator_queue;
using ::bot3::control_loops::intake_queue;
using ::frc971::wpilib::BufferedPcm;
using ::frc971::wpilib::BufferedSolenoid;
using ::frc971::wpilib::LoopOutputHandler;
using ::frc971::wpilib::JoystickSender;
using ::frc971::wpilib::GyroSender;
using ::frc971::wpilib::HallEffect;

namespace bot3 {
namespace wpilib {

double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*4x*/) *
         ::bot3::control_loops::kDrivetrainEncoderRatio *
         (4 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

double elevator_translate(int32_t in) {
  return static_cast<double>(in) / (512.0 /*cpr*/ * 4.0 /*4x*/) *
         ::bot3::control_loops::kElevEncoderRatio * (2 * M_PI /*radians*/) *
         ::bot3::control_loops::kElevChainReduction *
         ::bot3::control_loops::kElevGearboxOutputRadianDistance;
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
  }

  void set_right_encoder(::std::unique_ptr<Encoder> right_encoder) {
    right_encoder_ = ::std::move(right_encoder);
  }

  // Elevator setters.
  void set_elevator_encoder(::std::unique_ptr<Encoder> encoder) {
    filter_.Add(encoder.get());
    elevator_encoder_ = ::std::move(encoder);
  }

  void set_elevator_zeroing_hall_effect(::std::unique_ptr<HallEffect> hall) {
    zeroing_hall_effect_ = ::std::move(hall);
  }

  void operator()() {
    LOG(INFO, "In sensor reader thread\n");
    ::aos::SetCurrentThreadName("SensorReader");

    my_pid_ = getpid();
    ds_ = DriverStation::GetInstance();

    LOG(INFO, "Things are now started\n");

    ::aos::SetCurrentThreadRealtimePriority(kPriority);
    while (run_) {
      ::aos::time::PhasedLoopXMS(5, 4000);
      RunIteration();
    }
  }

  void RunIteration() {
    // General
    {
      auto new_state = ::aos::robot_state.MakeMessage();

      new_state->reader_pid = my_pid_;
      new_state->outputs_enabled = ds_->IsSysActive();
      new_state->browned_out = ds_->IsSysBrownedOut();

      new_state->is_3v3_active = ControllerPower::GetEnabled3V3();
      new_state->is_5v_active = ControllerPower::GetEnabled5V();
      new_state->voltage_3v3 = ControllerPower::GetVoltage3V3();
      new_state->voltage_5v = ControllerPower::GetVoltage5V();

      new_state->voltage_roborio_in = ControllerPower::GetInputVoltage();
      new_state->voltage_battery = ds_->GetBatteryVoltage();

      LOG_STRUCT(DEBUG, "robot_state", *new_state);

      new_state.Send();
    }

    // Drivetrain
    {
      auto drivetrain_message = drivetrain_queue.position.MakeMessage();
      drivetrain_message->right_encoder =
          -drivetrain_translate(right_encoder_->GetRaw());
      drivetrain_message->left_encoder =
          drivetrain_translate(left_encoder_->GetRaw());

      drivetrain_message.Send();
    }

    // Elevator
    {
      // Update control loop positions.
      auto elevator_message = elevator_queue.position.MakeMessage();
      elevator_message->encoder =
          elevator_translate(elevator_encoder_->GetRaw());
      elevator_message->bottom_hall_effect =
          zeroing_hall_effect_->Get();

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
  static const int kPriority = 30;
  static const int kInterruptPriority = 55;

  int32_t my_pid_;
  DriverStation *ds_;

  ::std::unique_ptr<Encoder> left_encoder_, right_encoder_, elevator_encoder_;
  ::std::unique_ptr<HallEffect> zeroing_hall_effect_;

  ::std::atomic<bool> run_{true};
  DigitalGlitchFilter filter_;
};

// Writes out our pneumatic outputs.
class SolenoidWriter {
 public:
  SolenoidWriter(const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm)
      : pcm_(pcm),
        elevator_(".bot3.control_loops.elevator_queue.output"),
        intake_(".bot3.control_loops.intake_queue.output") {}

  void set_pressure_switch(::std::unique_ptr<DigitalSource> pressure_switch) {
    pressure_switch_ = ::std::move(pressure_switch);
  }

  void set_compressor_relay(::std::unique_ptr<Relay> compressor_relay) {
    compressor_relay_ = ::std::move(compressor_relay);
  }

  void set_elevator_passive_support(
      ::std::unique_ptr<BufferedSolenoid> elevator_passive_support) {
    elevator_passive_support_ = ::std::move(elevator_passive_support);
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
    ::aos::SetCurrentThreadRealtimePriority(30);

    while (run_) {
      ::aos::time::PhasedLoopXMS(20, 1000);

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

  ::std::unique_ptr<DigitalSource> pressure_switch_;
  ::std::unique_ptr<Relay> compressor_relay_;

  ::aos::Queue<::bot3::control_loops::ElevatorQueue::Output> elevator_;
  ::aos::Queue<::bot3::control_loops::IntakeQueue::Output> intake_;

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
    ::bot3::control_loops::drivetrain_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::bot3::control_loops::drivetrain_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    left_drivetrain_talon_->Set(queue->left_voltage / 12.0);
    right_drivetrain_talon_->Set(-queue->right_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "drivetrain output too old\n");
    left_drivetrain_talon_->Disable();
    right_drivetrain_talon_->Disable();
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
    ::bot3::control_loops::elevator_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::bot3::control_loops::elevator_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    elevator_talon1_->Set(queue->elevator / 12.0);
    elevator_talon2_->Set(-queue->elevator / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Elevator output too old.\n");
    elevator_talon1_->Disable();
    elevator_talon2_->Disable();
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
    ::bot3::control_loops::intake_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::bot3::control_loops::intake_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    intake_talon1_->Set(queue->intake / 12.0);
    intake_talon2_->Set(-queue->intake / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Intake output too old.\n");
    intake_talon1_->Disable();
    intake_talon2_->Disable();
  }

  ::std::unique_ptr<Talon> intake_talon1_;
  ::std::unique_ptr<Talon> intake_talon2_;
};

// TODO(brian): Replace this with ::std::make_unique once all our toolchains
// have support.
template <class T, class... U>
std::unique_ptr<T> make_unique(U &&... u) {
  return std::unique_ptr<T>(new T(std::forward<U>(u)...));
}

class WPILibRobot : public RobotBase {
 public:
  ::std::unique_ptr<Encoder> encoder(int index) {
    return make_unique<Encoder>(10 + index * 2, 11 + index * 2, false,
                                Encoder::k4X);
  }
  virtual void StartCompetition() {
    ::aos::InitNRT();
    ::aos::SetCurrentThreadName("StartCompetition");

    JoystickSender joystick_sender;
    ::std::thread joystick_thread(::std::ref(joystick_sender));

    SensorReader reader;
    LOG(INFO, "Creating the reader\n");

    reader.set_elevator_encoder(encoder(6));
    reader.set_elevator_zeroing_hall_effect(
        make_unique<HallEffect>(6));

    reader.set_left_encoder(encoder(0));
    reader.set_right_encoder(encoder(1));

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

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_pressure_switch(make_unique<DigitalInput>(9));
    solenoid_writer.set_compressor_relay(make_unique<Relay>(0));
    // TODO (jasmine): Find solenoid numbers
    solenoid_writer.set_elevator_passive_support(pcm->MakeSolenoid(0));
    solenoid_writer.set_elevator_can_support(pcm->MakeSolenoid(1));
    solenoid_writer.set_intake_claw(pcm->MakeSolenoid(2));
    ::std::thread solenoid_thread(::std::ref(solenoid_writer));
    // TODO(comran): Find talon/encoder numbers ^^^

    // Wait forever. Not much else to do...
    PCHECK(select(0, nullptr, nullptr, nullptr, nullptr));

    LOG(ERROR, "Exiting WPILibRobot\n");

    joystick_sender.Quit();
    joystick_thread.join();
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

    solenoid_writer.Quit();
    solenoid_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace wpilib
}  // namespace bot3

START_ROBOT_CLASS(::bot3::wpilib::WPILibRobot);
