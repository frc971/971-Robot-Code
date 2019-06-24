#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <mutex>
#include <thread>

#include "frc971/wpilib/ahal/AnalogInput.h"
#include "frc971/wpilib/ahal/Compressor.h"
#include "frc971/wpilib/ahal/DigitalInput.h"
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
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/drivetrain_writer.h"
#include "frc971/wpilib/gyro_sender.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/logging.q.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/pdp_fetcher.h"
#include "frc971/wpilib/sensor_reader.h"
#include "y2014_bot3/control_loops/drivetrain/drivetrain_base.h"
#include "y2014_bot3/control_loops/rollers/rollers.h"
#include "y2014_bot3/control_loops/rollers/rollers.q.h"

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
using namespace frc;
using aos::make_unique;

namespace y2014_bot3 {
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
class SensorReader : public ::frc971::wpilib::SensorReader {
 public:
  SensorReader(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::SensorReader(event_loop),
        rollers_position_sender_(
            event_loop->MakeSender<
                ::y2014_bot3::control_loops::RollersQueue::Position>(
                ".y2014_bot3.control_loops.rollers_queue.position")) {}

  void RunIteration() {
    // Drivetrain
    {
      auto drivetrain_message = drivetrain_queue.position.MakeMessage();
      drivetrain_message->right_encoder =
          -drivetrain_translate(drivetrain_right_encoder_->GetRaw());
      drivetrain_message->left_encoder =
          drivetrain_translate(drivetrain_left_encoder_->GetRaw());
      drivetrain_message->left_speed =
          drivetrain_velocity_translate(drivetrain_left_encoder_->GetPeriod());
      drivetrain_message->right_speed =
          drivetrain_velocity_translate(drivetrain_right_encoder_->GetPeriod());

      drivetrain_message.Send();
    }

    // Rollers
    {
      auto rollers_message = rollers_position_sender_.MakeMessage();
      rollers_message.Send();
    }
  }

 private:
  ::aos::Sender<::y2014_bot3::control_loops::RollersQueue::Position>
      rollers_position_sender_;
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
                                        ::aos::monotonic_clock::now(),
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

  ::aos::Queue<::frc971::control_loops::DrivetrainQueue::Output> drivetrain_;
  ::aos::Queue<::y2014_bot3::control_loops::RollersQueue::Output> rollers_;

  ::std::atomic<bool> run_{true};
};

// Writes out rollers voltages.
class RollersWriter : public LoopOutputHandler {
 public:
  RollersWriter(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::LoopOutputHandler(event_loop) {}

  void set_rollers_front_intake_talon(::std::unique_ptr<Talon> t_left,
                                      ::std::unique_ptr<Talon> t_right) {
    rollers_front_left_intake_talon_ = ::std::move(t_left);
    rollers_front_right_intake_talon_ = ::std::move(t_right);
  }

  void set_rollers_back_intake_talon(::std::unique_ptr<Talon> t_left,
                                     ::std::unique_ptr<Talon> t_right) {
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
    rollers_front_left_intake_talon_->SetSpeed(queue->front_intake_voltage /
                                               12.0);
    rollers_front_right_intake_talon_->SetSpeed(
        -(queue->front_intake_voltage / 12.0));
    rollers_back_left_intake_talon_->SetSpeed(queue->back_intake_voltage /
                                              12.0);
    rollers_back_right_intake_talon_->SetSpeed(
        -(queue->back_intake_voltage / 12.0));
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

class WPILibRobot : public ::frc971::wpilib::WPILibRobotBase {
 public:
  ::std::unique_ptr<Encoder> make_encoder(int index) {
    return make_unique<Encoder>(10 + index * 2, 11 + index * 2, false,
                                Encoder::k4X);
  }
  void Run() override {
    ::aos::InitNRT();
    ::aos::SetCurrentThreadName("StartCompetition");

    ::aos::ShmEventLoop event_loop;

    JoystickSender joystick_sender(&event_loop);
    ::std::thread joystick_thread(::std::ref(joystick_sender));

    ::frc971::wpilib::PDPFetcher pdp_fetcher(&event_loop);
    ::std::thread pdp_fetcher_thread(::std::ref(pdp_fetcher));

    // TODO(comran): IO ports are placeholders at the moment, so match them to
    // the robot before turning on.

    // Sensors
    SensorReader reader(&event_loop);
    reader.set_drivetrain_left_encoder(make_encoder(4));
    reader.set_drivetrain_right_encoder(make_encoder(5));

    ::std::thread reader_thread(::std::ref(reader));
    GyroSender gyro_sender(&event_loop);
    ::std::thread gyro_thread(::std::ref(gyro_sender));

    // Outputs
    ::frc971::wpilib::DrivetrainWriter drivetrain_writer(&event_loop);
    drivetrain_writer.set_left_controller0(
        ::std::unique_ptr<Talon>(new Talon(5)), true);
    drivetrain_writer.set_right_controller0(
        ::std::unique_ptr<Talon>(new Talon(2)), false);
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    RollersWriter rollers_writer(&event_loop);
    rollers_writer.set_rollers_front_intake_talon(
        ::std::unique_ptr<Talon>(new Talon(3)),
        ::std::unique_ptr<Talon>(new Talon(7)));
    rollers_writer.set_rollers_back_intake_talon(
        ::std::unique_ptr<Talon>(new Talon(1)),
        ::std::unique_ptr<Talon>(new Talon(6)));

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
}  // namespace y2014_bot3

AOS_ROBOT_CLASS(::y2014_bot3::wpilib::WPILibRobot);
