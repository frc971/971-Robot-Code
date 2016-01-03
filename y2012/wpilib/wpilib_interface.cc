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
#include "PowerDistributionPanel.h"
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

#include "y2012/control_loops/drivetrain/drivetrain.q.h"
#include "y2012/control_loops/accessories/accessories.q.h"

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::y2012::control_loops::drivetrain_queue;
using ::y2012::control_loops::accessories_queue;

namespace y2012 {
namespace wpilib {

template <class T, class... U>
std::unique_ptr<T> make_unique(U &&... u) {
  return std::unique_ptr<T>(new T(std::forward<U>(u)...));
}

double drivetrain_translate(int32_t in) {
  return -static_cast<double>(in) /
         (256.0 /*cpr*/ * 4.0 /*4x*/) *
         1 *
         (3.5 /*wheel diameter*/ * 2.54 / 100.0 * M_PI) * 2.0 / 2.0;
}

double drivetrain_velocity_translate(double in) {
  return (1.0 / in) / 256.0 /*cpr*/ *
         1 *
         (3.5 /*wheel diameter*/ * 2.54 / 100.0 * M_PI) * 2.0 / 2.0;
}

static const double kMaximumEncoderPulsesPerSecond =
    5600.0 /* free speed RPM */ * 14.0 / 48.0 /* bottom gear reduction */ *
    18.0 / 32.0 /* big belt reduction */ *
    18.0 / 66.0 /* top gear reduction */ * 48.0 / 18.0 /* encoder gears */ /
    60.0 /* seconds / minute */ * 256.0 /* CPR */;

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

    accessories_queue.position.MakeMessage().Send();
  }

  void Quit() { run_ = false; }

 private:
  int32_t my_pid_;
  DriverStation *ds_;

  ::std::unique_ptr<Encoder> drivetrain_left_encoder_;
  ::std::unique_ptr<Encoder> drivetrain_right_encoder_;
  ::std::atomic<bool> run_{true};
};

class SolenoidWriter {
 public:
  SolenoidWriter(const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm)
      : pcm_(pcm),
        drivetrain_(".y2012.control_loops.drivetrain_queue.output"),
        accessories_(".y2012.control_loops.accessories_queue.output") {}

  void set_drivetrain_high(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    drivetrain_high_ = ::std::move(s);
  }

  void set_drivetrain_low(
      ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    drivetrain_low_ = ::std::move(s);
  }

  void set_s1(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    s1_ = ::std::move(s);
  }

  void set_s2(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    s2_ = ::std::move(s);
  }

  void set_s3(::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s) {
    s3_ = ::std::move(s);
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
        accessories_.FetchLatest();
        if (accessories_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *accessories_);
					s1_->Set(accessories_->solenoids[0]);
					s2_->Set(accessories_->solenoids[1]);
					s3_->Set(accessories_->solenoids[2]);
        }
      }

      {
        drivetrain_.FetchLatest();
        if (drivetrain_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *drivetrain_);
          const bool high = drivetrain_->left_high || drivetrain_->right_high;
          drivetrain_high_->Set(high);
          drivetrain_low_->Set(!high);
        }
      }

      {
        ::frc971::wpilib::PneumaticsToLog to_log;
        pcm_->Flush();
        to_log.read_solenoids = pcm_->GetAll();
        LOG_STRUCT(DEBUG, "pneumatics info", to_log);
      }
    }
  }

  void Quit() { run_ = false; }

 private:
  const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm_;

  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> drivetrain_high_;
  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> drivetrain_low_;
  ::std::unique_ptr<::frc971::wpilib::BufferedSolenoid> s1_, s2_, s3_;

  ::std::unique_ptr<Compressor> compressor_;

  ::aos::Queue<::y2012::control_loops::DrivetrainQueue::Output> drivetrain_;
  ::aos::Queue<::y2012::control_loops::AccessoriesQueue::Message> accessories_;

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
    ::y2012::control_loops::drivetrain_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2012::control_loops::drivetrain_queue.output;
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

class AccessoriesWriter : public ::frc971::wpilib::LoopOutputHandler {
 public:
  void set_talon1(::std::unique_ptr<Talon> t) {
    talon1_ = ::std::move(t);
  }

  void set_talon2(::std::unique_ptr<Talon> t) {
    talon2_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::y2012::control_loops::accessories_queue.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::y2012::control_loops::accessories_queue.output;
    talon1_->Set(queue->sticks[0]);
    talon2_->Set(queue->sticks[1]);
    LOG_STRUCT(DEBUG, "will output", *queue);
  }

  virtual void Stop() override {
    LOG(WARNING, "shooter output too old\n");
    talon1_->Disable();
    talon2_->Disable();
  }

  ::std::unique_ptr<Talon> talon1_, talon2_;
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

    SensorReader reader;

    reader.set_drivetrain_left_encoder(make_encoder(0));
    reader.set_drivetrain_right_encoder(make_encoder(1));

    ::std::thread reader_thread(::std::ref(reader));

    DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_left_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(3)));
    drivetrain_writer.set_right_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(4)));
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    ::y2012::wpilib::AccessoriesWriter accessories_writer;
    accessories_writer.set_talon1(::std::unique_ptr<Talon>(new Talon(5)));
    accessories_writer.set_talon2(::std::unique_ptr<Talon>(new Talon(6)));
    ::std::thread accessories_writer_thread(::std::ref(accessories_writer));

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_drivetrain_high(pcm->MakeSolenoid(0));
    solenoid_writer.set_drivetrain_low(pcm->MakeSolenoid(2));
    solenoid_writer.set_s1(pcm->MakeSolenoid(1));
    solenoid_writer.set_s2(pcm->MakeSolenoid(3));
    solenoid_writer.set_s3(pcm->MakeSolenoid(4));

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
    reader.Quit();
    reader_thread.join();

    drivetrain_writer.Quit();
    drivetrain_writer_thread.join();
    accessories_writer.Quit();
    accessories_writer_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace wpilib
}  // namespace y2012


AOS_ROBOT_CLASS(::y2012::wpilib::WPILibRobot);
