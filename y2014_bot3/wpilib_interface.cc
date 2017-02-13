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

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2014_bot3/control_loops/drivetrain/drivetrain_base.h"
#include "y2014_bot3/control_loops/rollers/rollers.q.h"
#include "y2014_bot3/autonomous/auto.q.h"
#include "y2014_bot3/control_loops/rollers/rollers.h"

#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/gyro_sender.h"
#include "frc971/wpilib/logging.q.h"
#include "frc971/wpilib/wpilib_interface.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/dma.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::aos::util::SimpleLogInterval;
using ::frc971::control_loops::drivetrain_queue;
using ::y2014_bot3::control_loops::rollers_queue;
using ::frc971::wpilib::BufferedPcm;
using ::frc971::wpilib::BufferedSolenoid;
using ::frc971::wpilib::LoopOutputHandler;
using ::frc971::wpilib::JoystickSender;
using ::frc971::wpilib::GyroSender;

namespace frc971 {
namespace wpilib {

double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*4x*/) *
         ::y2014_bot3::control_loops::drivetrain::kDrivetrainEncoderRatio *
         (4 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

double drivetrain_velocity_translate(double in) {
  return (1.0 / in) / 256.0 /*cpr*/ *
         ::y2014_bot3::control_loops::drivetrain::kDrivetrainEncoderRatio *
         (4 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

// Reads in our inputs. (sensors, voltages, etc.)
class SensorReader {
 public:
  SensorReader() {}

  void set_drivetrain_left_encoder(::std::unique_ptr<Encoder> encoder) {
    drivetrain_left_encoder_ = ::std::move(encoder);
    drivetrain_left_encoder_->SetMaxPeriod(0.005);
  }

  void set_drivetrain_right_encoder(::std::unique_ptr<Encoder> encoder) {
    drivetrain_right_encoder_ = ::std::move(encoder);
    drivetrain_right_encoder_->SetMaxPeriod(0.005);
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

    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                        ::std::chrono::milliseconds(4));

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
          drivetrain_translate(drivetrain_right_encoder_->GetRaw());
      drivetrain_message->left_encoder =
          -drivetrain_translate(drivetrain_left_encoder_->GetRaw());
      drivetrain_message->left_speed =
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod());
      drivetrain_message->right_speed =
          drivetrain_velocity_translate(drivetrain_right_encoder_->GetPeriod());

      drivetrain_message.Send();
    }

    // Rollers
    {
      auto rollers_message = rollers_queue.position.MakeMessage();
      rollers_message.Send();
    }
  }

  void Quit() { run_ = false; }

 private:
  int32_t my_pid_;
  DriverStation *ds_;

  ::std::unique_ptr<Encoder> drivetrain_left_encoder_;
  ::std::unique_ptr<Encoder> drivetrain_right_encoder_;

  ::std::atomic<bool> run_{true};
};

// Writes out our pneumatic outputs.
class SolenoidWriter {
 public:
  SolenoidWriter(const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm)
      : pcm_(pcm),
        drivetrain_(".frc971.control_loops.drivetrain_queue.output"),
        rollers_(".y2014_bot3.control_loops.rollers_queue.output") {}

  void set_pressure_switch(::std::unique_ptr<DigitalInput> pressure_switch) {
    pressure_switch_ = ::std::move(pressure_switch);
  }

  void set_compressor_relay(::std::unique_ptr<Relay> compressor_relay) {
    compressor_relay_ = ::std::move(compressor_relay);
  }

  void set_drivetrain_left(::std::unique_ptr<BufferedSolenoid> s) {
    drivetrain_left_ = ::std::move(s);
  }

  void set_drivetrain_right(::std::unique_ptr<BufferedSolenoid> s) {
    drivetrain_right_ = ::std::move(s);
  }

  void set_rollers_front(::std::unique_ptr<BufferedSolenoid> s) {
    rollers_front_ = ::std::move(s);
  }

  void set_rollers_back(::std::unique_ptr<BufferedSolenoid> s) {
    rollers_back_ = ::std::move(s);
  }

  void operator()() {
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

      // Drivetrain
      {
        drivetrain_.FetchLatest();
        if (drivetrain_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *drivetrain_);
          drivetrain_left_->Set(drivetrain_->left_high);
          drivetrain_right_->Set(drivetrain_->right_high);
        }
      }

      // Intake
      {
        rollers_.FetchLatest();
        if (rollers_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *rollers_);
          rollers_front_->Set(rollers_->front_extended);
          rollers_back_->Set(rollers_->back_extended);
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

  ::std::unique_ptr<BufferedSolenoid> drivetrain_left_, drivetrain_right_;
  ::std::unique_ptr<BufferedSolenoid> rollers_front_, rollers_back_;

  ::std::unique_ptr<DigitalInput> pressure_switch_;
  ::std::unique_ptr<Relay> compressor_relay_;

  ::aos::Queue<::frc971::control_loops::DrivetrainQueue::Output>
      drivetrain_;
  ::aos::Queue<::y2014_bot3::control_loops::RollersQueue::Output> rollers_;

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
    ::frc971::control_loops::drivetrain_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::frc971::control_loops::drivetrain_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    left_drivetrain_talon_->SetSpeed(-queue->left_voltage / 12.0);
    right_drivetrain_talon_->SetSpeed(queue->right_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "drivetrain output too old\n");
    left_drivetrain_talon_->SetDisabled();
    right_drivetrain_talon_->SetDisabled();
  }

  ::std::unique_ptr<Talon> left_drivetrain_talon_;
  ::std::unique_ptr<Talon> right_drivetrain_talon_;
};

// Writes out rollers voltages.
class RollersWriter : public LoopOutputHandler {
 public:
  void set_rollers_front_intake_talon(::std::unique_ptr<Talon> t_left, ::std::unique_ptr<Talon> t_right) {
    rollers_front_left_intake_talon_ = ::std::move(t_left);
    rollers_front_right_intake_talon_ = ::std::move(t_right);
  }

  void set_rollers_back_intake_talon(::std::unique_ptr<Talon> t_left, ::std::unique_ptr<Talon> t_right) {
    rollers_back_left_intake_talon_ = ::std::move(t_left);
    rollers_back_right_intake_talon_ = ::std::move(t_right);
  }

  void set_rollers_low_goal_talon(::std::unique_ptr<Talon> t) {
    rollers_low_goal_talon_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2014_bot3::control_loops::rollers_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2014_bot3::control_loops::rollers_queue.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    rollers_front_left_intake_talon_->SetSpeed(queue->front_intake_voltage / 12.0);
    rollers_front_right_intake_talon_->SetSpeed(-(queue->front_intake_voltage / 12.0));
    rollers_back_left_intake_talon_->SetSpeed(queue->back_intake_voltage / 12.0);
    rollers_back_right_intake_talon_->SetSpeed(-(queue->back_intake_voltage / 12.0));
    rollers_low_goal_talon_->SetSpeed(queue->low_goal_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Intake output too old\n");
    rollers_front_left_intake_talon_->SetDisabled();
    rollers_front_right_intake_talon_->SetDisabled();
    rollers_back_left_intake_talon_->SetDisabled();
    rollers_back_right_intake_talon_->SetDisabled();
    rollers_low_goal_talon_->SetDisabled();
  }

  ::std::unique_ptr<Talon> rollers_front_left_intake_talon_,
      rollers_back_left_intake_talon_, rollers_front_right_intake_talon_,
      rollers_back_right_intake_talon_, rollers_low_goal_talon_;
};

// TODO(brian): Replace this with ::std::make_unique once all our toolchains
// have support.
template <class T, class... U>
std::unique_ptr<T> make_unique(U &&... u) {
  return std::unique_ptr<T>(new T(std::forward<U>(u)...));
}

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<Encoder> make_encoder(int index) {
    return make_unique<Encoder>(10 + index * 2, 11 + index * 2, false,
                                Encoder::k4X);
  }
  void Run() override {
    ::aos::InitNRT();
    ::aos::SetCurrentThreadName("StartCompetition");

    JoystickSender joystick_sender;
    ::std::thread joystick_thread(::std::ref(joystick_sender));

    ::frc971::wpilib::PDPFetcher pdp_fetcher;
    ::std::thread pdp_fetcher_thread(::std::ref(pdp_fetcher));

    //TODO(comran): IO ports are placeholders at the moment, so match them to
    // the robot before turning on.

    // Sensors
    SensorReader reader;
    reader.set_drivetrain_left_encoder(make_encoder(4));
    reader.set_drivetrain_right_encoder(make_encoder(5));

    ::std::thread reader_thread(::std::ref(reader));
    GyroSender gyro_sender;
    ::std::thread gyro_thread(::std::ref(gyro_sender));

    // Outputs
    DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_left_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(5)));
    drivetrain_writer.set_right_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(2)));
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    RollersWriter rollers_writer;
    rollers_writer.set_rollers_front_intake_talon(
        ::std::unique_ptr<Talon>(new Talon(3)), ::std::unique_ptr<Talon>(new Talon(7)));
    rollers_writer.set_rollers_back_intake_talon(
        ::std::unique_ptr<Talon>(new Talon(1)), ::std::unique_ptr<Talon>(new Talon(6)));

    rollers_writer.set_rollers_low_goal_talon(
        ::std::unique_ptr<Talon>(new Talon(4)));
    ::std::thread rollers_writer_thread(::std::ref(rollers_writer));

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_drivetrain_left(pcm->MakeSolenoid(6));
    solenoid_writer.set_drivetrain_right(pcm->MakeSolenoid(5));
    solenoid_writer.set_rollers_front(pcm->MakeSolenoid(2));
    solenoid_writer.set_rollers_back(pcm->MakeSolenoid(4));

    // Don't change the following IDs.
    solenoid_writer.set_pressure_switch(make_unique<DigitalInput>(9));
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

    rollers_writer.Quit();
    rollers_writer_thread.join();

    solenoid_writer.Quit();
    solenoid_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace wpilib
}  // namespace frc971

AOS_ROBOT_CLASS(::frc971::wpilib::WPILibRobot);
