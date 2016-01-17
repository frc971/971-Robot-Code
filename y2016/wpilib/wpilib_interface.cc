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

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2016/constants.h"

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

double drivetrain_translate(int32_t in) {
  return -static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*4x*/) *
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
      0.5 +
      0.5 * (in_high - static_cast<float>(k.high_gear_middle)) /
          static_cast<float>(k.high_gear_high - k.high_gear_middle);

  // Return low when we are below 1/2, and high when we are above 1/2.
  if (low_ratio + high_ratio < 1.0) {
    return low_ratio;
  } else {
    return high_ratio;
  }
}

// TODO(constants): Update.
static const double kMaximumEncoderPulsesPerSecond =
    5600.0 /* free speed RPM */ * 14.0 / 48.0 /* bottom gear reduction */ *
    18.0 / 32.0 /* big belt reduction */ * 18.0 /
    66.0 /* top gear reduction */ * 48.0 / 18.0 /* encoder gears */ /
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

  // All of the DMA-related set_* calls must be made before this, and it doesn't
  // hurt to do all of them.

  // TODO(comran): Add 2016 things down below for dma synchronization.
  void set_dma(::std::unique_ptr<DMA> dma) {
    dma_synchronizer_.reset(
        new ::frc971::wpilib::DMASynchronizer(::std::move(dma)));
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

    dma_synchronizer_->RunIteration();
  }

  void Quit() { run_ = false; }

 private:
  int32_t my_pid_;
  DriverStation *ds_;

  ::std::unique_ptr<::frc971::wpilib::DMASynchronizer> dma_synchronizer_;

  ::std::unique_ptr<Encoder> drivetrain_left_encoder_;
  ::std::unique_ptr<Encoder> drivetrain_right_encoder_;
  ::std::unique_ptr<AnalogInput> low_left_drive_hall_;
  ::std::unique_ptr<AnalogInput> high_left_drive_hall_;
  ::std::unique_ptr<AnalogInput> low_right_drive_hall_;
  ::std::unique_ptr<AnalogInput> high_right_drive_hall_;

  ::std::atomic<bool> run_{true};
  DigitalGlitchFilter encoder_filter_, hall_filter_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm)
      : pcm_(pcm),
        drivetrain_(".frc971.control_loops.drivetrain_queue.output") {}

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

  ::std::unique_ptr<DigitalInput> pressure_switch_;
  ::std::unique_ptr<Relay> compressor_relay_;

  ::aos::Queue<::frc971::control_loops::DrivetrainQueue::Output> drivetrain_;

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
    reader.set_drivetrain_left_encoder(make_encoder(0));
    reader.set_drivetrain_right_encoder(make_encoder(1));
    reader.set_high_left_drive_hall(make_unique<AnalogInput>(1));
    reader.set_low_left_drive_hall(make_unique<AnalogInput>(0));
    reader.set_high_right_drive_hall(make_unique<AnalogInput>(2));
    reader.set_low_right_drive_hall(make_unique<AnalogInput>(3));

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

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_drivetrain_left(pcm->MakeSolenoid(6));
    solenoid_writer.set_drivetrain_right(pcm->MakeSolenoid(7));

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
    solenoid_writer.Quit();
    solenoid_thread.join();

    ::aos::Cleanup();
  }
};

}  // namespace wpilib
}  // namespace y2016

AOS_ROBOT_CLASS(::y2016::wpilib::WPILibRobot);
