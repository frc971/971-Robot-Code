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
#include "aos/linux_code/init.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/constants.h"
#include "frc971/shifter_hall_effect.h"

#include "frc971/wpilib/hall_effect.h"
#include "frc971/wpilib/joystick_sender.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/buffered_solenoid.h"
#include "frc971/wpilib/buffered_pcm.h"
#include "frc971/wpilib/gyro_sender.h"

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
using ::frc971::control_loops::drivetrain;
using ::aos::util::WrappingCounter;

namespace frc971 {
namespace wpilib {

class priority_mutex {
 public:
  typedef pthread_mutex_t *native_handle_type;

  // TODO(austin): Write a test case for the mutex, and make the constructor
  // constexpr.
  priority_mutex() {
    pthread_mutexattr_t attr;
#ifdef NDEBUG
#error "Won't let assert_perror be no-op ed"
#endif
    // Turn on priority inheritance.
    assert_perror(pthread_mutexattr_init(&attr));
    assert_perror(pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_NORMAL));
    assert_perror(pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT));

    assert_perror(pthread_mutex_init(native_handle(), &attr));

    assert_perror(pthread_mutexattr_destroy(&attr));
  }

  ~priority_mutex() { pthread_mutex_destroy(&handle_); }

  void lock() { assert_perror(pthread_mutex_lock(&handle_)); }
  bool try_lock() {
    int ret = pthread_mutex_trylock(&handle_);
    if (ret == 0) {
      return true;
    } else if (ret == EBUSY) {
      return false;
    } else {
      assert_perror(ret);
    }
  }
  void unlock() { assert_perror(pthread_mutex_unlock(&handle_)); }

  native_handle_type native_handle() { return &handle_; }

 private:
  DISALLOW_COPY_AND_ASSIGN(priority_mutex);
  pthread_mutex_t handle_;
};

// TODO(brian): Split this out into a separate file once DMA is in.
class EdgeCounter {
 public:
  EdgeCounter(int priority, Encoder *encoder, HallEffect *input,
              priority_mutex *mutex)
      : priority_(priority),
        encoder_(encoder),
        input_(input),
        mutex_(mutex),
        run_(true),
        any_interrupt_count_(0) {
    thread_.reset(new ::std::thread(::std::ref(*this)));
  }

  // Waits for interrupts, locks the mutex, and updates the internal state.
  // Updates the any_interrupt_count count when the interrupt comes in without
  // the lock.
  void operator()() {
    ::aos::SetCurrentThreadName("EdgeCounter_" +
                                ::std::to_string(input_->GetChannel()));

    input_->RequestInterrupts();
    input_->SetUpSourceEdge(true, true);

    {
      ::std::unique_lock<priority_mutex> mutex_guard(*mutex_);
      current_value_ = input_->GetHall();
    }

    ::aos::SetCurrentThreadRealtimePriority(priority_);
    InterruptableSensorBase::WaitResult result = InterruptableSensorBase::kBoth;
    while (run_) {
      result = input_->WaitForInterrupt(
          0.1, result != InterruptableSensorBase::kTimeout);
      if (result == InterruptableSensorBase::kTimeout) {
        continue;
      }
      ++any_interrupt_count_;

      ::std::unique_lock<priority_mutex> mutex_guard(*mutex_);
      int32_t encoder_value = encoder_->GetRaw();
      bool hall_value = input_->GetHall();
      if (current_value_ != hall_value) {
        if (hall_value) {
          ++positive_interrupt_count_;
          last_positive_encoder_value_ = encoder_value;
        } else {
          ++negative_interrupt_count_;
          last_negative_encoder_value_ = encoder_value;
        }
      } else {
        LOG(WARNING, "Detected spurious edge on %d.  Dropping it.\n",
            input_->GetChannel());
      }

      current_value_ = hall_value;
    }
  }

  // Updates the internal hall effect value given this new observation.
  // The mutex provided at construction time must be held during this operation.
  void set_polled_value(bool value) {
    polled_value_ = value;
    bool miss_match = (value != current_value_);
    if (miss_match && last_miss_match_) {
      current_value_ = value;
      last_miss_match_ = false;
    } else {
      last_miss_match_ = miss_match;
    }
  }

  // Signals the thread to quit next time it gets an interrupt.
  void Quit() {
    run_ = false;
    thread_->join();
  }

  // Returns the total number of interrupts since construction time.  This
  // should be done without the mutex held.
  int any_interrupt_count() const { return any_interrupt_count_; }
  // Returns the current interrupt edge counts and encoder values.
  // The mutex provided at construction time must be held during this operation.
  int positive_interrupt_count() const { return positive_interrupt_count_; }
  int negative_interrupt_count() const { return negative_interrupt_count_; }
  int32_t last_positive_encoder_value() const {
    return last_positive_encoder_value_;
  }
  int32_t last_negative_encoder_value() const {
    return last_negative_encoder_value_;
  }
  // Returns the current polled value.
  bool polled_value() const { return polled_value_; }

 private:
  int priority_;
  Encoder *encoder_;
  HallEffect *input_;
  priority_mutex *mutex_;
  ::std::atomic<bool> run_;

  ::std::atomic<int> any_interrupt_count_;

  // The following variables represent the current state.  They must be
  // synchronized by mutex_;
  bool current_value_ = false;
  bool polled_value_ = false;
  bool last_miss_match_ = true;
  int positive_interrupt_count_ = 0;
  int negative_interrupt_count_ = 0;
  int32_t last_positive_encoder_value_ = 0;
  int32_t last_negative_encoder_value_ = 0;

  ::std::unique_ptr<::std::thread> thread_;
};

// This class will synchronize sampling edges on a bunch of HallEffects with
// the periodic poll.
//
// The data is provided to subclasses by calling SaveState when the state is
// consistent and ready.
//
// TODO(brian): Split this out into a separate file once DMA is in.
template <int num_sensors>
class PeriodicHallSynchronizer {
 public:
  PeriodicHallSynchronizer(
      const char *name, int priority, int interrupt_priority,
      ::std::unique_ptr<Encoder> encoder,
      ::std::array<::std::unique_ptr<HallEffect>, num_sensors> *sensors)
      : name_(name),
        priority_(priority),
        encoder_(::std::move(encoder)),
        run_(true) {
    for (int i = 0; i < num_sensors; ++i) {
      sensors_[i] = ::std::move((*sensors)[i]);
      edge_counters_[i] = ::std::unique_ptr<EdgeCounter>(new EdgeCounter(
          interrupt_priority, encoder_.get(), sensors_[i].get(), &mutex_));
    }
  }

  const char *name() const { return name_.c_str(); }

  void StartThread() { thread_.reset(new ::std::thread(::std::ref(*this))); }

  // Called when the state is consistent and up to date.
  virtual void SaveState() = 0;

  // Starts a sampling iteration.  See RunIteration for usage.
  void StartIteration() {
    // Start by capturing the current interrupt counts.
    for (int i = 0; i < num_sensors; ++i) {
      interrupt_counts_[i] = edge_counters_[i]->any_interrupt_count();
    }

    {
      // Now, update the encoder and sensor values.
      ::std::unique_lock<priority_mutex> mutex_guard(mutex_);
      encoder_value_ = encoder_->GetRaw();
      for (int i = 0; i < num_sensors; ++i) {
        edge_counters_[i]->set_polled_value(sensors_[i]->GetHall());
      }
    }
  }

  // Attempts to finish a sampling iteration.  See RunIteration for usage.
  // Returns true if the iteration succeeded, and false otherwise.
  bool TryFinishingIteration() {
    // Make sure no interrupts have occurred while we were waiting.  If they
    // have, we are in an inconsistent state and need to try again.
    ::std::unique_lock<priority_mutex> mutex_guard(mutex_);
    bool retry = false;
    for (int i = 0; i < num_sensors; ++i) {
      retry = retry || (interrupt_counts_[i] !=
                        edge_counters_[i]->any_interrupt_count());
    }
    if (!retry) {
      SaveState();
      return true;
    }
    LOG(WARNING, "Got an interrupt while sampling encoder %s, retrying\n",
        name());
    return false;
  }

  void RunIteration() {
    while (true) {
      StartIteration();

      // Wait more than the amount of time it takes for a digital input change
      // to go from visible to software to having triggered an interrupt.
      ::aos::time::SleepFor(::aos::time::Time::InUS(120));

      if (TryFinishingIteration()) {
        return;
      }
    }
  }

  void operator()() {
    ::aos::SetCurrentThreadName("HallSync" + ::std::to_string(num_sensors));
    ::aos::SetCurrentThreadRealtimePriority(priority_);
    while (run_) {
      ::aos::time::PhasedLoopXMS(10, 9000);
      RunIteration();
    }
  }

  void Quit() {
    run_ = false;
    for (int i = 0; i < num_sensors; ++i) {
      edge_counters_[i]->Quit();
    }
    if (thread_) {
      thread_->join();
    }
  }

 protected:
  // These values are only safe to fetch from inside SaveState()
  int32_t encoder_value() const { return encoder_value_; }
  ::std::array<::std::unique_ptr<EdgeCounter>, num_sensors> &edge_counters() {
    return edge_counters_;
  }

 private:
  // A descriptive name for error messages.
  ::std::string name_;
  // The priority of the polling thread.
  int priority_;
  // The Encoder to sample.
  ::std::unique_ptr<Encoder> encoder_;
  // A list of all the digital inputs.
  ::std::array<::std::unique_ptr<HallEffect>, num_sensors> sensors_;
  // The mutex used to synchronize all the state.
  priority_mutex mutex_;
  ::std::atomic<bool> run_;

  // The state.
  // The current encoder value.
  int32_t encoder_value_ = 0;
  // The current edge counters.
  ::std::array<::std::unique_ptr<EdgeCounter>, num_sensors> edge_counters_;

  ::std::unique_ptr<::std::thread> thread_;
  ::std::array<int, num_sensors> interrupt_counts_;
};

double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) /
         (256.0 /*cpr*/ * 2.0 /*2x.  Stupid WPILib*/) *
         (18.0 / 50.0 /*output stage*/) * (56.0 / 30.0 /*encoder gears*/)
         // * constants::GetValues().drivetrain_encoder_ratio
         *
         (3.5 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

class SensorReader {
 public:
  SensorReader()
      : left_encoder_(new Encoder(11, 10, false, Encoder::k2X)),   // E0
        right_encoder_(new Encoder(13, 12, false, Encoder::k2X)),  // E1
        run_(true) {
    filter_.SetPeriodNanoSeconds(100000);
  }

  void operator()() {
    ::aos::SetCurrentThreadName("SensorReader");

    const int kPriority = 30;
    //const int kInterruptPriority = 55;

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

    // TODO(austin): Calibrate the shifter constants again.
    // TODO(sensors): Hook up the new dog position sensors.
    drivetrain.position.MakeWithBuilder()
        .right_encoder(drivetrain_translate(right_encoder_->GetRaw()))
        .left_encoder(-drivetrain_translate(left_encoder_->GetRaw()))
        .left_shifter_position(0)
        .right_shifter_position(0)
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
  ::std::unique_ptr<AnalogInput> auto_selector_analog_;

  ::std::unique_ptr<Encoder> left_encoder_;
  ::std::unique_ptr<Encoder> right_encoder_;

  ::std::atomic<bool> run_;
  DigitalGlitchFilter filter_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(const ::std::unique_ptr<BufferedPcm> &pcm)
      : pcm_(pcm), drivetrain_(".frc971.control_loops.drivetrain.output") {}

  void set_drivetrain_left(::std::unique_ptr<BufferedSolenoid> s) {
    drivetrain_left_ = ::std::move(s);
  }

  void set_drivetrain_right(::std::unique_ptr<BufferedSolenoid> s) {
    drivetrain_right_ = ::std::move(s);
  }

  void operator()() {
    ::aos::SetCurrentThreadName("Solenoids");
    ::aos::SetCurrentThreadRealtimePriority(30);

    while (run_) {
      ::aos::time::PhasedLoopXMS(20, 1000);

      {
        drivetrain_.FetchLatest();
        if (drivetrain_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *drivetrain_);
          drivetrain_left_->Set(drivetrain_->left_high);
          drivetrain_right_->Set(drivetrain_->right_high);
        }
      }

      pcm_->Flush();
    }
  }

  void Quit() { run_ = false; }

 private:
  const ::std::unique_ptr<BufferedPcm> &pcm_;
  ::std::unique_ptr<BufferedSolenoid> drivetrain_left_;
  ::std::unique_ptr<BufferedSolenoid> drivetrain_right_;

  ::aos::Queue<::frc971::control_loops::Drivetrain::Output> drivetrain_;

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
    ::frc971::control_loops::drivetrain.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::frc971::control_loops::drivetrain.output;
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

}  // namespace wpilib
}  // namespace frc971

class WPILibRobot : public RobotBase {
 public:
  virtual void StartCompetition() {
    ::aos::InitNRT();
    ::aos::SetCurrentThreadName("StartCompetition");

    ::frc971::wpilib::JoystickSender joystick_sender;
    ::std::thread joystick_thread(::std::ref(joystick_sender));
    ::frc971::wpilib::SensorReader reader;
    ::std::thread reader_thread(::std::ref(reader));
    ::frc971::wpilib::GyroSender gyro_sender;
    ::std::thread gyro_thread(::std::ref(gyro_sender));
    ::std::unique_ptr<Compressor> compressor(new Compressor());
    compressor->SetClosedLoopControl(true);

    ::frc971::wpilib::DrivetrainWriter drivetrain_writer;
    drivetrain_writer.set_left_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(5)));
    drivetrain_writer.set_right_drivetrain_talon(
        ::std::unique_ptr<Talon>(new Talon(2)));
    ::std::thread drivetrain_writer_thread(::std::ref(drivetrain_writer));

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    ::frc971::wpilib::SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_drivetrain_left(pcm->MakeSolenoid(6));
    solenoid_writer.set_drivetrain_right(pcm->MakeSolenoid(7));
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


START_ROBOT_CLASS(WPILibRobot);
