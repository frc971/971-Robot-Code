#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include <thread>
#include <mutex>
#include <functional>

#include "aos/common/controls/output_check.q.h"
#include "aos/common/controls/sensor_generation.q.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/messages/robot_state.q.h"
#include "aos/common/time.h"
#include "aos/common/util/log_interval.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/util/wrapping_counter.h"
#include "aos/common/stl_mutex.h"
#include "aos/linux_code/init.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/fridge/fridge.q.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/constants.h"
#include "frc971/shifter_hall_effect.h"

#include "frc971/wpilib/hall_effect.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/gyro_sender.h"
#include "frc971/wpilib/dma_edge_counting.h"
#include "frc971/wpilib/interrupt_edge_counting.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"

#include "Encoder.h"
#include "Talon.h"
#include "DriverStation.h"
#include "AnalogInput.h"
#include "Compressor.h"
#include "RobotBase.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::aos::util::SimpleLogInterval;
using ::frc971::control_loops::drivetrain_queue;
using ::aos::util::WrappingCounter;

namespace frc971 {
namespace wpilib {

double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) /
         (256.0 /*cpr*/ * 4.0 /*4x*/) *
         (20.0 / 50.0 /*output stage*/) *
         // * constants::GetValues().drivetrain_encoder_ratio
         (4 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

double arm_translate(int32_t in) {
  return static_cast<double>(in) /
          (512.0 /*cpr*/ * 4.0 /*4x*/) *
          (14.0 / 17.0 /*output sprockets*/) *
          (18.0 / 48.0 /*encoder pulleys*/) *
          (2 * M_PI /*radians*/);
}

double arm_pot_translate(double voltage) {
  return voltage /
          (14.0 / 17.0 /*output sprockets*/) *
          (5.0 /*volts*/ / 5.0 /*turns*/) *
          (2 * M_PI /*radians*/);
}

double elevator_translate(int32_t in) {
  return static_cast<double>(in) /
          (512.0 /*cpr*/ * 4.0 /*4x*/) *
          (14.0 / 84.0 /*output stage*/) *
          (32 * 5 / 2.54 / 10 /*pulley circumference (in)*/);
}

double elevator_pot_translate(double voltage) {
  return voltage /
          (32 * 5 / 2.54 / 10 /*pulley circumference (in)*/) *
          (5.0 /*volts*/ / 5.0 /*turns*/);
}

double claw_translate(int32_t in) {
  return static_cast<double>(in) /
          (512.0 /*cpr*/ * 4.0 /*4x*/) *
          (16.0 / 72.0 /*output sprockets*/) *
          (2 * M_PI /*radians*/);
}

double claw_pot_translate(double voltage) {
  return voltage /
          (16.0 / 72.0 /*output sprockets*/) *
          (5.0 /*volts*/ / 5.0 /*turns*/) *
          (2 * M_PI /*radians*/);
}

class SensorReader {
 public:
  SensorReader() {
    filter_.SetPeriodNanoSeconds(100000);
  }

  void set_left_encoder(::std::unique_ptr<Encoder> left_encoder) {
    left_encoder_ = ::std::move(left_encoder);
  }

  void set_right_encoder(::std::unique_ptr<Encoder> right_encoder) {
    right_encoder_ = ::std::move(right_encoder);
  }

  void operator()() {
    ::aos::SetCurrentThreadName("SensorReader");

    static const int kPriority = 30;
    //static const int kInterruptPriority = 55;

    ::aos::SetCurrentThreadRealtimePriority(kPriority);
    while (run_) {
      ::aos::time::PhasedLoopXMS(5, 9000);
      RunIteration();
    }
  }

  void RunIteration() {
    DriverStation *ds = DriverStation::GetInstance();

    if (ds->IsSysActive()) {
      auto message = ::aos::controls::output_check_received.MakeMessage();
      // TODO(brians): Actually read a pulse value from the roboRIO.
      message->pwm_value = 0;
      message->pulse_length = -1;
      LOG_STRUCT(DEBUG, "received", *message);
      message.Send();
    }

    drivetrain_queue.position.MakeWithBuilder()
        .right_encoder(drivetrain_translate(right_encoder_->GetRaw()))
        .left_encoder(-drivetrain_translate(left_encoder_->GetRaw()))
        .battery_voltage(ds->GetBatteryVoltage())
        .Send();

    // Signal that we are alive.
    ::aos::controls::sensor_generation.MakeWithBuilder()
        .reader_pid(getpid())
        .cape_resets(0)
        .Send();
  }

  void Quit() { run_ = false; }

 private:
  ::std::unique_ptr<Encoder> left_encoder_;
  ::std::unique_ptr<Encoder> right_encoder_;

  ::std::atomic<bool> run_{true};
  DigitalGlitchFilter filter_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(const ::std::unique_ptr<BufferedPcm> &pcm)
      : pcm_(pcm),
      fridge_(".frc971.control_loops.fridge.output"),
      claw_(".frc971.control_loops.claw.output") {}

  void set_fridge_grabbers_top_front(::std::unique_ptr<BufferedSolenoid> s) {
    fridge_grabbers_top_front_ = ::std::move(s);
  }

  void set_fridge_grabbers_top_back(::std::unique_ptr<BufferedSolenoid> s) {
    fridge_grabbers_top_back_ = ::std::move(s);
  }

  void set_fridge_grabbers_bottom_front(
      ::std::unique_ptr<BufferedSolenoid> s) {
    fridge_grabbers_bottom_front_ = ::std::move(s);
  }

  void set_fridge_grabbers_bottom_back(
      ::std::unique_ptr<BufferedSolenoid> s) {
    fridge_grabbers_bottom_back_ = ::std::move(s);
  }

  void set_claw_pinchers(::std::unique_ptr<BufferedSolenoid> s) {
    claw_pinchers_ = ::std::move(s);
  }

  void operator()() {
    ::aos::SetCurrentThreadName("Solenoids");
    ::aos::SetCurrentThreadRealtimePriority(30);

    while (run_) {
      ::aos::time::PhasedLoopXMS(20, 1000);

      {
        fridge_.FetchLatest();
        if (fridge_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *fridge_);
          fridge_grabbers_top_front_->Set(fridge_->grabbers.top_front);
          fridge_grabbers_top_back_->Set(fridge_->grabbers.top_back);
          fridge_grabbers_bottom_front_->Set(fridge_->grabbers.bottom_front);
          fridge_grabbers_bottom_back_->Set(fridge_->grabbers.bottom_back);
        }
      }

      {
        claw_.FetchLatest();
        if (claw_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *claw_);
          claw_pinchers_->Set(claw_->rollers_closed);
        }
      }

      pcm_->Flush();
    }
  }

  void Quit() { run_ = false; }

 private:
  const ::std::unique_ptr<BufferedPcm> &pcm_;
  ::std::unique_ptr<BufferedSolenoid> fridge_grabbers_top_front_;
  ::std::unique_ptr<BufferedSolenoid> fridge_grabbers_top_back_;
  ::std::unique_ptr<BufferedSolenoid> fridge_grabbers_bottom_front_;
  ::std::unique_ptr<BufferedSolenoid> fridge_grabbers_bottom_back_;
  ::std::unique_ptr<BufferedSolenoid> claw_pinchers_;

  ::aos::Queue<::frc971::control_loops::FridgeQueue::Output> fridge_;
  ::aos::Queue<::frc971::control_loops::ClawQueue::Output> claw_;

  ::std::atomic<bool> run_{true};
};

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
    ::frc971::control_loops::drivetrain_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::frc971::control_loops::drivetrain_queue.output;
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

class FridgeWriter : public LoopOutputHandler {
 public:
  void set_left_arm_talon(::std::unique_ptr<Talon> t) {
    left_arm_talon_ = ::std::move(t);
  }

  void set_right_arm_talon(::std::unique_ptr<Talon> t) {
    right_arm_talon_ = ::std::move(t);
  }

  void set_left_elevator_talon(::std::unique_ptr<Talon> t) {
    left_elevator_talon_ = ::std::move(t);
  }

  void set_right_elevator_talon(::std::unique_ptr<Talon> t) {
    right_elevator_talon_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::frc971::control_loops::fridge_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::frc971::control_loops::fridge_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    left_arm_talon_->Set(-queue->left_arm / 12.0);
    right_arm_talon_->Set(queue->right_arm / 12.0);
    left_elevator_talon_->Set(-queue->left_elevator / 12.0);
    right_elevator_talon_->Set(queue->right_elevator / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Fridge output too old.\n");
    left_arm_talon_->Disable();
    right_arm_talon_->Disable();
    left_elevator_talon_->Disable();
    right_elevator_talon_->Disable();
  }

  ::std::unique_ptr<Talon> left_arm_talon_;
  ::std::unique_ptr<Talon> right_arm_talon_;
  ::std::unique_ptr<Talon> left_elevator_talon_;
  ::std::unique_ptr<Talon> right_elevator_talon_;
};

class ClawWriter : public LoopOutputHandler {
 public:
  void set_intake_talon(::std::unique_ptr<Talon> t) {
    intake_talon_ = ::std::move(t);
  }

  void set_wrist_talon(::std::unique_ptr<Talon> t) {
    wrist_talon_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::frc971::control_loops::claw_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::frc971::control_loops::claw_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    intake_talon_->Set(queue->intake_voltage / 12.0);
    wrist_talon_->Set(queue->voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Claw output too old.\n");
    intake_talon_->Disable();
    wrist_talon_->Disable();
  }

  ::std::unique_ptr<Talon> intake_talon_;
  ::std::unique_ptr<Talon> wrist_talon_;
};

// TODO(brian): Replace this with ::std::make_unique once all our toolchains
// have support.
template <class T, class... U>
std::unique_ptr<T> make_unique(U &&... u) {
  return std::unique_ptr<T>(new T(std::forward<U>(u)...));
}

class WPILibRobot : public RobotBase {
 public:
  virtual void StartCompetition() {
    ::aos::InitNRT();
    ::aos::SetCurrentThreadName("StartCompetition");

    JoystickSender joystick_sender;
    ::std::thread joystick_thread(::std::ref(joystick_sender));
    ::std::unique_ptr<Compressor> compressor(new Compressor());
    compressor->SetClosedLoopControl(true);

    SensorReader reader;
    // TODO(sensors): Replace all the 99s with real port numbers.
    reader.set_left_encoder(make_unique<Encoder>(99, 99, false, Encoder::k4X));
    reader.set_right_encoder(make_unique<Encoder>(99, 99, false, Encoder::k4X));
    ::std::thread reader_thread(::std::ref(reader));
    GyroSender gyro_sender;
    ::std::thread gyro_thread(::std::ref(gyro_sender));

    DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_left_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(5)));
    drivetrain_writer.set_right_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(2)));
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    // TODO(sensors): Get real PWM output and relay numbers for the fridge and
    // claw.
    FridgeWriter fridge_writer;
    fridge_writer.set_left_arm_talon(
        ::std::unique_ptr<Talon>(new Talon(99)));
    fridge_writer.set_right_arm_talon(
        ::std::unique_ptr<Talon>(new Talon(99)));
    fridge_writer.set_left_elevator_talon(
        ::std::unique_ptr<Talon>(new Talon(99)));
    fridge_writer.set_right_elevator_talon(
        ::std::unique_ptr<Talon>(new Talon(99)));
    ::std::thread fridge_writer_thread(::std::ref(fridge_writer));

    ClawWriter claw_writer;
    claw_writer.set_intake_talon(
        ::std::unique_ptr<Talon>(new Talon(99)));
    claw_writer.set_wrist_talon(
        ::std::unique_ptr<Talon>(new Talon(99)));
    ::std::thread claw_writer_thread(::std::ref(claw_writer));

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_fridge_grabbers_top_front(pcm->MakeSolenoid(99));
    solenoid_writer.set_fridge_grabbers_top_back(pcm->MakeSolenoid(99));
    solenoid_writer.set_fridge_grabbers_bottom_front(pcm->MakeSolenoid(99));
    solenoid_writer.set_fridge_grabbers_bottom_back(pcm->MakeSolenoid(99));
    solenoid_writer.set_claw_pinchers(pcm->MakeSolenoid(99));
    ::std::thread solenoid_thread(::std::ref(solenoid_writer));

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
    solenoid_writer.Quit();
    solenoid_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace wpilib
}  // namespace frc971


START_ROBOT_CLASS(::frc971::wpilib::WPILibRobot);
