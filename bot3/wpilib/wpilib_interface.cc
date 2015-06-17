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

#include "frc971/wpilib/hall_effect.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/gyro_sender.h"
#include "frc971/wpilib/dma_edge_counting.h"
#include "frc971/wpilib/interrupt_edge_counting.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/logging.q.h"
#include "bot3/control_loops/drivetrain/drivetrain.h"

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
using ::frc971::wpilib::DMAEncoderAndPotentiometer;
using ::frc971::PotAndIndexPosition;
using ::frc971::wpilib::InterruptEncoderAndPotentiometer;
using ::frc971::wpilib::DMASynchronizer;
using ::frc971::wpilib::BufferedPcm;
using ::frc971::wpilib::LoopOutputHandler;
using ::frc971::wpilib::JoystickSender;
using ::frc971::wpilib::GyroSender;

namespace bot3 {
namespace wpilib {

double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) /
         (256.0 /*cpr*/ * 4.0 /*4x*/) *
         ::bot3::control_loops::kDrivetrainEncoderRatio *
         (4 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

// TODO(comran): Check/update the values below for the bot3.
static const double kMaximumEncoderPulsesPerSecond =
    19500.0 /* free speed RPM */ * 12.0 / 56.0 /* belt reduction */ /
    60.0 /* seconds / minute */ * 256.0 /* CPR */ *
    4.0 /* index pulse = 1/4 cycle */;

class SensorReader {
 public:
  SensorReader() {
    // Set it to filter out anything shorter than 1/4 of the minimum pulse width
    // we should ever see.
    filter_.SetPeriodNanoSeconds(
        static_cast<int>(1 / 4.0 / kMaximumEncoderPulsesPerSecond * 1e9 + 0.5));
  }

  void set_left_encoder(::std::unique_ptr<Encoder> left_encoder) {
    left_encoder_ = ::std::move(left_encoder);
  }

  void set_right_encoder(::std::unique_ptr<Encoder> right_encoder) {
    right_encoder_ = ::std::move(right_encoder);
  }

  // All of the DMA-related set_* calls must be made before this, and it doesn't
  // hurt to do all of them.
  // TODO(comran): Do we still need dma?
  void set_dma(::std::unique_ptr<DMA> dma) {
    dma_synchronizer_.reset(new DMASynchronizer(::std::move(dma)));
  }

  void operator()() {
    LOG(INFO, "In sensor reader thread\n");
    ::aos::SetCurrentThreadName("SensorReader");

    my_pid_ = getpid();
    ds_ = DriverStation::GetInstance();

    dma_synchronizer_->Start();
    LOG(INFO, "Things are now started\n");

    ::aos::SetCurrentThreadRealtimePriority(kPriority);
    while (run_) {
      ::aos::time::PhasedLoopXMS(5, 4000);
      RunIteration();
    }
  }

  void RunIteration() {
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

    {
      auto drivetrain_message = drivetrain_queue.position.MakeMessage();
      drivetrain_message->right_encoder =
          -drivetrain_translate(right_encoder_->GetRaw());
      drivetrain_message->left_encoder =
          drivetrain_translate(left_encoder_->GetRaw());

      drivetrain_message.Send();
    }

    dma_synchronizer_->RunIteration();
  }

  void Quit() { run_ = false; }

 private:
  static const int kPriority = 30;
  static const int kInterruptPriority = 55;

  int32_t my_pid_;
  DriverStation *ds_;

  void CopyPotAndIndexPosition(
      const DMAEncoderAndPotentiometer &encoder, PotAndIndexPosition *position,
      ::std::function<double(int32_t)> encoder_translate,
      ::std::function<double(double)> potentiometer_translate, bool reverse,
      double potentiometer_offset) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->encoder =
        multiplier * encoder_translate(encoder.polled_encoder_value());
    position->pot = multiplier * potentiometer_translate(
                                     encoder.polled_potentiometer_voltage()) +
                    potentiometer_offset;
    position->latched_encoder =
        multiplier * encoder_translate(encoder.last_encoder_value());
    position->latched_pot =
        multiplier *
            potentiometer_translate(encoder.last_potentiometer_voltage()) +
        potentiometer_offset;
    position->index_pulses = encoder.index_posedge_count();
  }

  void CopyPotAndIndexPosition(
      const InterruptEncoderAndPotentiometer &encoder,
      PotAndIndexPosition *position,
      ::std::function<double(int32_t)> encoder_translate,
      ::std::function<double(double)> potentiometer_translate, bool reverse,
      double potentiometer_offset) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->encoder =
        multiplier * encoder_translate(encoder.encoder()->GetRaw());
    position->pot = multiplier * potentiometer_translate(
                                     encoder.potentiometer()->GetVoltage()) +
                    potentiometer_offset;
    position->latched_encoder =
        multiplier * encoder_translate(encoder.last_encoder_value());
    position->latched_pot =
        multiplier *
            potentiometer_translate(encoder.last_potentiometer_voltage()) +
        potentiometer_offset;
    position->index_pulses = encoder.index_posedge_count();
  }

  ::std::unique_ptr<DMASynchronizer> dma_synchronizer_;

  ::std::unique_ptr<Encoder> left_encoder_;
  ::std::unique_ptr<Encoder> right_encoder_;

  ::std::atomic<bool> run_{true};
  DigitalGlitchFilter filter_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(const ::std::unique_ptr<::frc971::wpilib::BufferedPcm> &pcm)
      : pcm_(pcm) {}

  void set_pressure_switch(::std::unique_ptr<DigitalSource> pressure_switch) {
    pressure_switch_ = ::std::move(pressure_switch);
  }

  void set_compressor_relay(::std::unique_ptr<Relay> compressor_relay) {
    compressor_relay_ = ::std::move(compressor_relay);
  }

  void operator()() {
    ::aos::SetCurrentThreadName("Solenoids");
    ::aos::SetCurrentThreadRealtimePriority(30);

    while (run_) {
      ::aos::time::PhasedLoopXMS(20, 1000);

      ::aos::joystick_state.FetchLatest();

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
  const ::std::unique_ptr<BufferedPcm> &pcm_;
  ::std::unique_ptr<DigitalSource> pressure_switch_;
  ::std::unique_ptr<Relay> compressor_relay_;

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

    // TODO(comran): Find talon/encoder numbers.
    reader.set_left_encoder(encoder(2));
    reader.set_right_encoder(encoder(3));
    reader.set_dma(make_unique<DMA>());
    ::std::thread reader_thread(::std::ref(reader));
    GyroSender gyro_sender;
    ::std::thread gyro_thread(::std::ref(gyro_sender));

    DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_left_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(8)));
    drivetrain_writer.set_right_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(0)));
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_pressure_switch(make_unique<DigitalInput>(9));
    solenoid_writer.set_compressor_relay(make_unique<Relay>(0));
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
}  // namespace bot3


START_ROBOT_CLASS(::bot3::wpilib::WPILibRobot);
