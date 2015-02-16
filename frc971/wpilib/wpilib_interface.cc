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

#include "frc971/constants.h"
#include "frc971/control_loops/control_loops.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/fridge/fridge.q.h"
#include "frc971/control_loops/claw/claw.q.h"

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
#include "dma.h"
#include "ControllerPower.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::aos::util::SimpleLogInterval;
using ::frc971::control_loops::drivetrain_queue;
using ::frc971::control_loops::fridge_queue;
using ::frc971::control_loops::claw_queue;

namespace frc971 {
namespace wpilib {

double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) /
         (256.0 /*cpr*/ * 4.0 /*4x*/) *
         constants::GetValues().drivetrain_encoder_ratio *
         (4 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

double arm_translate(int32_t in) {
  return -static_cast<double>(in) /
          (512.0 /*cpr*/ * 4.0 /*4x*/) *
          constants::GetValues().arm_encoder_ratio *
          (2 * M_PI /*radians*/);
}

double arm_potentiometer_translate(double voltage) {
  return voltage *
          constants::GetValues().arm_pot_ratio *
          (5.0 /*turns*/ / 5.0 /*volts*/) *
          (2 * M_PI /*radians*/);
}

double elevator_translate(int32_t in) {
  return static_cast<double>(in) /
          (512.0 /*cpr*/ * 4.0 /*4x*/) *
          constants::GetValues().elev_encoder_ratio *
          (2 * M_PI /*radians*/) *
          constants::GetValues().elev_distance_per_radian;
}

double elevator_potentiometer_translate(double voltage) {
  return -voltage *
          constants::GetValues().elev_pot_ratio *
          (2 * M_PI /*radians*/) *
          constants::GetValues().elev_distance_per_radian *
          (5.0 /*turns*/ / 5.0 /*volts*/);
}

double claw_translate(int32_t in) {
  return static_cast<double>(in) /
          (512.0 /*cpr*/ * 4.0 /*4x*/) *
          constants::GetValues().claw_encoder_ratio *
          (2 * M_PI /*radians*/);
}

double claw_potentiometer_translate(double voltage) {
  return -voltage *
          constants::GetValues().claw_pot_ratio *
          (5.0 /*turns*/ / 5.0 /*volts*/) *
          (2 * M_PI /*radians*/);
}

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

  void set_arm_left_encoder(::std::unique_ptr<Encoder> encoder) {
    filter_.Add(encoder.get());
    arm_left_encoder_.set_encoder(::std::move(encoder));
  }

  void set_arm_left_index(::std::unique_ptr<DigitalSource> index) {
    filter_.Add(index.get());
    arm_left_encoder_.set_index(::std::move(index));
  }

  void set_arm_left_potentiometer(
      ::std::unique_ptr<AnalogInput> potentiometer) {
    arm_left_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_arm_right_encoder(::std::unique_ptr<Encoder> encoder) {
    filter_.Add(encoder.get());
    arm_right_encoder_.set_encoder(::std::move(encoder));
  }

  void set_arm_right_index(::std::unique_ptr<DigitalSource> index) {
    filter_.Add(index.get());
    arm_right_encoder_.set_index(::std::move(index));
  }

  void set_arm_right_potentiometer(
      ::std::unique_ptr<AnalogInput> potentiometer) {
    arm_right_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_elevator_left_encoder(::std::unique_ptr<Encoder> encoder) {
    filter_.Add(encoder.get());
    elevator_left_encoder_.set_encoder(::std::move(encoder));
  }

  void set_elevator_left_index(::std::unique_ptr<DigitalSource> index) {
    filter_.Add(index.get());
    elevator_left_encoder_.set_index(::std::move(index));
  }

  void set_elevator_left_potentiometer(
      ::std::unique_ptr<AnalogInput> potentiometer) {
    elevator_left_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_elevator_right_encoder(::std::unique_ptr<Encoder> encoder) {
    filter_.Add(encoder.get());
    elevator_right_encoder_.set_encoder(::std::move(encoder));
  }

  void set_elevator_right_index(::std::unique_ptr<DigitalSource> index) {
    filter_.Add(index.get());
    elevator_right_encoder_.set_index(::std::move(index));
  }

  void set_elevator_right_potentiometer(
      ::std::unique_ptr<AnalogInput> potentiometer) {
    elevator_right_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_wrist_encoder(::std::unique_ptr<Encoder> encoder) {
    filter_.Add(encoder.get());
    wrist_encoder_.set_encoder(::std::move(encoder));
  }

  void set_wrist_index(::std::unique_ptr<DigitalSource> index) {
    filter_.Add(index.get());
    wrist_encoder_.set_index(::std::move(index));
  }

  void set_wrist_potentiometer(::std::unique_ptr<AnalogInput> potentiometer) {
    wrist_encoder_.set_potentiometer(::std::move(potentiometer));
  }

  void set_left_encoder(::std::unique_ptr<Encoder> left_encoder) {
    left_encoder_ = ::std::move(left_encoder);
  }

  void set_right_encoder(::std::unique_ptr<Encoder> right_encoder) {
    right_encoder_ = ::std::move(right_encoder);
  }

  // All of the DMA-related set_* calls must be made before this, and it doesn't
  // hurt to do all of them.
  void set_dma(::std::unique_ptr<DMA> dma) {
    dma_synchronizer_.reset(new DMASynchronizer(::std::move(dma)));
    dma_synchronizer_->Add(&arm_left_encoder_);
    dma_synchronizer_->Add(&elevator_left_encoder_);
    dma_synchronizer_->Add(&arm_right_encoder_);
    dma_synchronizer_->Add(&elevator_right_encoder_);
  }

  void operator()() {
    LOG(INFO, "In sensor reader thread\n");
    ::aos::SetCurrentThreadName("SensorReader");

    my_pid_ = getpid();
    ds_ = DriverStation::GetInstance();

    wrist_encoder_.Start();
    dma_synchronizer_->Start();
    LOG(INFO, "Things are now started\n");

    ::aos::SetCurrentThreadRealtimePriority(kPriority);
    while (run_) {
      ::aos::time::PhasedLoopXMS(5, 4000);
      RunIteration();
    }

    wrist_encoder_.Stop();
  }

  void RunIteration() {
    {
      auto new_state = ::aos::robot_state.MakeMessage();

      new_state->outputs_enabled = ds_->IsSysActive();
      new_state->browned_out = ds_->IsSysBrownedOut();

      new_state->is_3v3_active = ControllerPower::GetEnabled3V3();
      new_state->is_5v_active = ControllerPower::GetEnabled5V();
      new_state->voltage_3v3 = ControllerPower::GetVoltage3V3();
      new_state->voltage_5v = ControllerPower::GetVoltage5V();

      new_state->voltage_roborio_in = ControllerPower::GetInputVoltage();
      new_state->voltage_battery = ds_->GetBatteryVoltage();

      new_state.Send();
    }

    {
      auto drivetrain_message = drivetrain_queue.position.MakeMessage();
      drivetrain_message->right_encoder =
          drivetrain_translate(right_encoder_->GetRaw());
      drivetrain_message->left_encoder =
          -drivetrain_translate(left_encoder_->GetRaw());

      drivetrain_message.Send();
    }

    dma_synchronizer_->RunIteration();

    const auto &values = constants::GetValues();

    {
      auto fridge_message = fridge_queue.position.MakeMessage();
      CopyPotAndIndexPosition(arm_left_encoder_, &fridge_message->arm.left,
                              arm_translate, arm_potentiometer_translate, false,
                              values.fridge.left_arm_potentiometer_offset);
      CopyPotAndIndexPosition(
          arm_right_encoder_, &fridge_message->arm.right, arm_translate,
          arm_potentiometer_translate, true,
          values.fridge.right_arm_potentiometer_offset);
      CopyPotAndIndexPosition(
          elevator_left_encoder_, &fridge_message->elevator.left,
          elevator_translate, elevator_potentiometer_translate, false,
          values.fridge.left_elevator_potentiometer_offset);
      CopyPotAndIndexPosition(
          elevator_right_encoder_, &fridge_message->elevator.right,
          elevator_translate, elevator_potentiometer_translate, true,
          values.fridge.right_elevator_potentiometer_offset);
      fridge_message.Send();
    }

    {
      auto claw_message = claw_queue.position.MakeMessage();
      CopyPotAndIndexPosition(wrist_encoder_, &claw_message->joint,
                              claw_translate, claw_potentiometer_translate,
                              false, values.claw.potentiometer_offset);
      claw_message.Send();
    }
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

  DMAEncoderAndPotentiometer arm_left_encoder_, arm_right_encoder_,
      elevator_left_encoder_, elevator_right_encoder_;

  InterruptEncoderAndPotentiometer wrist_encoder_{kInterruptPriority};

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
    left_arm_talon_->Set(queue->left_arm / 12.0);
    right_arm_talon_->Set(-queue->right_arm / 12.0);
    left_elevator_talon_->Set(queue->left_elevator / 12.0);
    right_elevator_talon_->Set(-queue->right_elevator / 12.0);
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
  void set_left_intake_talon(::std::unique_ptr<Talon> t) {
    left_intake_talon_ = ::std::move(t);
  }

  void set_right_intake_talon(::std::unique_ptr<Talon> t) {
    right_intake_talon_ = ::std::move(t);
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
    left_intake_talon_->Set(queue->intake_voltage / 12.0);
    right_intake_talon_->Set(queue->intake_voltage / 12.0);
    wrist_talon_->Set(queue->voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "Claw output too old.\n");
    left_intake_talon_->Disable();
    right_intake_talon_->Disable();
    wrist_talon_->Disable();
  }

  ::std::unique_ptr<Talon> left_intake_talon_;
  ::std::unique_ptr<Talon> right_intake_talon_;
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
  ::std::unique_ptr<Encoder> encoder(int index) {
    return make_unique<Encoder>(10 + index * 2, 11 + index * 2, false,
                                Encoder::k4X);
  }
  virtual void StartCompetition() {
    ::aos::InitNRT();
    ::aos::SetCurrentThreadName("StartCompetition");

    JoystickSender joystick_sender;
    ::std::thread joystick_thread(::std::ref(joystick_sender));
    // TODO(austin): Compressor needs to use a spike.
    ::std::unique_ptr<Compressor> compressor(new Compressor());
    compressor->SetClosedLoopControl(true);

    SensorReader reader;
    LOG(INFO, "Creating the reader\n");
    reader.set_arm_left_encoder(encoder(1));
    reader.set_arm_left_index(make_unique<DigitalInput>(1));
    reader.set_arm_left_potentiometer(make_unique<AnalogInput>(1));

    reader.set_arm_right_encoder(encoder(5));
    reader.set_arm_right_index(make_unique<DigitalInput>(5));
    reader.set_arm_right_potentiometer(make_unique<AnalogInput>(5));

    reader.set_elevator_left_encoder(encoder(0));
    reader.set_elevator_left_index(make_unique<DigitalInput>(0));
    reader.set_elevator_left_potentiometer(make_unique<AnalogInput>(0));

    reader.set_elevator_right_encoder(encoder(4));
    reader.set_elevator_right_index(make_unique<DigitalInput>(4));
    reader.set_elevator_right_potentiometer(make_unique<AnalogInput>(4));

    reader.set_wrist_encoder(encoder(6));
    reader.set_wrist_index(make_unique<DigitalInput>(6));
    reader.set_wrist_potentiometer(make_unique<AnalogInput>(6));

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

    // TODO(sensors): Get real PWM output and relay numbers for the fridge and
    // claw.
    FridgeWriter fridge_writer;
    fridge_writer.set_left_arm_talon(
        ::std::unique_ptr<Talon>(new Talon(6)));
    fridge_writer.set_right_arm_talon(
        ::std::unique_ptr<Talon>(new Talon(2)));
    fridge_writer.set_left_elevator_talon(
        ::std::unique_ptr<Talon>(new Talon(7)));
    fridge_writer.set_right_elevator_talon(
        ::std::unique_ptr<Talon>(new Talon(1)));
    ::std::thread fridge_writer_thread(::std::ref(fridge_writer));

    ClawWriter claw_writer;
    claw_writer.set_left_intake_talon(
        ::std::unique_ptr<Talon>(new Talon(5)));
    claw_writer.set_right_intake_talon(
        ::std::unique_ptr<Talon>(new Talon(3)));
    claw_writer.set_wrist_talon(
        ::std::unique_ptr<Talon>(new Talon(4)));
    ::std::thread claw_writer_thread(::std::ref(claw_writer));

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_fridge_grabbers_top_front(pcm->MakeSolenoid(1));
    solenoid_writer.set_fridge_grabbers_top_back(pcm->MakeSolenoid(1));
    solenoid_writer.set_fridge_grabbers_bottom_front(pcm->MakeSolenoid(2));
    solenoid_writer.set_fridge_grabbers_bottom_back(pcm->MakeSolenoid(3));
    solenoid_writer.set_claw_pinchers(pcm->MakeSolenoid(0));
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
