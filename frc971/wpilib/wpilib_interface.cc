#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include <thread>
#include <mutex>
#include <functional>

#include "aos/prime/output/motor_output.h"
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
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/shooter/shooter.q.h"
#include "frc971/constants.h"
#include "frc971/queues/other_sensors.q.h"
#include "frc971/queues/to_log.q.h"
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

static const double kVcc = 5.15;

float hall_translate(const constants::ShifterHallEffect &k, float in_low,
                     float in_high) {
  const float low_ratio =
      0.5 * (in_low - static_cast<float>(k.low_gear_low)) /
      static_cast<float>(k.low_gear_middle - k.low_gear_low);
  const float high_ratio =
      0.5 + 0.5 * (in_high - static_cast<float>(k.high_gear_middle)) /
                static_cast<float>(k.high_gear_high - k.high_gear_middle);

  // Return low when we are below 1/2, and high when we are above 1/2.
  if (low_ratio + high_ratio < 1.0) {
    return low_ratio;
  } else {
    return high_ratio;
  }
}

double claw_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) /
         (18.0 / 48.0 /*encoder gears*/) / (12.0 / 60.0 /*chain reduction*/) *
         (M_PI / 180.0) *
         2.0 /*TODO(austin): Debug this, encoders read too little*/;
}

double shooter_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) *
         16 /*sprocket teeth*/ * 0.375 /*chain pitch*/
         * (2.54 / 100.0 /*in to m*/);
}

// This class sends out half of the claw position state at 100 hz.
class HalfClawHallSynchronizer : public PeriodicHallSynchronizer<3> {
 public:
  // Constructs a HalfClawHallSynchronizer.
  //
  // priority is the priority of the polling thread.
  // interrupt_priority is the priority of the interrupt threads.
  // encoder is the encoder to read.
  // sensors is an array of hall effect sensors.  The sensors[0] is the front
  //   sensor, sensors[1] is the calibration sensor, sensors[2] is the back
  //   sensor.
  HalfClawHallSynchronizer(
      const char *name, int priority, int interrupt_priority,
      ::std::unique_ptr<Encoder> encoder,
      ::std::array<::std::unique_ptr<HallEffect>, 3> *sensors, bool reversed)
      : PeriodicHallSynchronizer<3>(name, priority, interrupt_priority,
                                    ::std::move(encoder), sensors),
        reversed_(reversed) {}

  void set_position(control_loops::HalfClawPosition *half_claw_position) {
    half_claw_position_ = half_claw_position;
  }

  // Saves the state so that it can be sent if it was synchronized.
  virtual void SaveState() {
    const auto &front = edge_counters()[0];
    half_claw_position_->front.current = front->polled_value();
    half_claw_position_->front.posedge_count =
        front->positive_interrupt_count();
    half_claw_position_->front.negedge_count =
        front->negative_interrupt_count();

    const auto &calibration = edge_counters()[1];
    half_claw_position_->calibration.current = calibration->polled_value();
    half_claw_position_->calibration.posedge_count =
        calibration->positive_interrupt_count();
    half_claw_position_->calibration.negedge_count =
        calibration->negative_interrupt_count();

    const auto &back = edge_counters()[2];
    half_claw_position_->back.current = back->polled_value();
    half_claw_position_->back.posedge_count = back->positive_interrupt_count();
    half_claw_position_->back.negedge_count = back->negative_interrupt_count();

    const double multiplier = reversed_ ? -1.0 : 1.0;

    half_claw_position_->position =
        multiplier * claw_translate(encoder_value());

    // We assume here that we can only have 1 sensor have a posedge per cycle.
    {
      half_claw_position_->posedge_value =
          last_half_claw_position_.posedge_value;
      int posedge_changes = 0;
      if (half_claw_position_->front.posedge_count !=
          last_half_claw_position_.front.posedge_count) {
        ++posedge_changes;
        half_claw_position_->posedge_value =
            multiplier * claw_translate(front->last_positive_encoder_value());
        LOG(INFO, "Got a front posedge\n");
      }

      if (half_claw_position_->back.posedge_count !=
          last_half_claw_position_.back.posedge_count) {
        ++posedge_changes;
        half_claw_position_->posedge_value =
            multiplier * claw_translate(back->last_positive_encoder_value());
        LOG(INFO, "Got a back posedge\n");
      }

      if (half_claw_position_->calibration.posedge_count !=
          last_half_claw_position_.calibration.posedge_count) {
        ++posedge_changes;
        half_claw_position_->posedge_value =
            multiplier *
            claw_translate(calibration->last_positive_encoder_value());
        LOG(INFO, "Got a calibration posedge\n");
      }

      if (posedge_changes > 1) {
        LOG(WARNING, "Found more than 1 positive edge on %s\n", name());
      }
    }

    {
      half_claw_position_->negedge_value =
          last_half_claw_position_.negedge_value;
      int negedge_changes = 0;
      if (half_claw_position_->front.negedge_count !=
          last_half_claw_position_.front.negedge_count) {
        ++negedge_changes;
        half_claw_position_->negedge_value =
            multiplier * claw_translate(front->last_negative_encoder_value());
        LOG(INFO, "Got a front negedge\n");
      }

      if (half_claw_position_->back.negedge_count !=
          last_half_claw_position_.back.negedge_count) {
        ++negedge_changes;
        half_claw_position_->negedge_value =
            multiplier * claw_translate(back->last_negative_encoder_value());
        LOG(INFO, "Got a back negedge\n");
      }

      if (half_claw_position_->calibration.negedge_count !=
          last_half_claw_position_.calibration.negedge_count) {
        ++negedge_changes;
        half_claw_position_->negedge_value =
            multiplier *
            claw_translate(calibration->last_negative_encoder_value());
        LOG(INFO, "Got a calibration negedge\n");
      }

      if (negedge_changes > 1) {
        LOG(WARNING, "Found more than 1 negative edge on %s\n", name());
      }
    }

    last_half_claw_position_ = *half_claw_position_;
  }

 private:
  control_loops::HalfClawPosition *half_claw_position_;
  control_loops::HalfClawPosition last_half_claw_position_;
  bool reversed_;
};

// This class sends out the shooter position state at 100 hz.
class ShooterHallSynchronizer : public PeriodicHallSynchronizer<2> {
 public:
  // Constructs a ShooterHallSynchronizer.
  //
  // priority is the priority of the polling thread.
  // interrupt_priority is the priority of the interrupt threads.
  // encoder is the encoder to read.
  // sensors is an array of hall effect sensors.  The sensors[0] is the proximal
  //   sensor, sensors[1] is the distal sensor.
  // shooter_plunger is the plunger.
  // shooter_latch is the latch.
  ShooterHallSynchronizer(
      int priority, int interrupt_priority, ::std::unique_ptr<Encoder> encoder,
      ::std::array<::std::unique_ptr<HallEffect>, 2> *sensors,
      ::std::unique_ptr<HallEffect> shooter_plunger,
      ::std::unique_ptr<HallEffect> shooter_latch)
      : PeriodicHallSynchronizer<2>("shooter", priority, interrupt_priority,
                                    ::std::move(encoder), sensors),
        shooter_plunger_(::std::move(shooter_plunger)),
        shooter_latch_(::std::move(shooter_latch)) {}

  // Saves the state so that it can be sent if it was synchronized.
  virtual void SaveState() {
    auto shooter_position =
        control_loops::shooter_queue_group.position.MakeMessage();

    shooter_position->plunger = shooter_plunger_->GetHall();
    shooter_position->latch = shooter_latch_->GetHall();
    shooter_position->position = shooter_translate(encoder_value());

    {
      const auto &proximal_edge_counter = edge_counters()[0];
      shooter_position->pusher_proximal.current =
          proximal_edge_counter->polled_value();
      shooter_position->pusher_proximal.posedge_count =
          proximal_edge_counter->positive_interrupt_count();
      shooter_position->pusher_proximal.negedge_count =
          proximal_edge_counter->negative_interrupt_count();
      shooter_position->pusher_proximal.posedge_value = shooter_translate(
          proximal_edge_counter->last_positive_encoder_value());
    }

    {
      const auto &distal_edge_counter = edge_counters()[1];
      shooter_position->pusher_distal.current =
          distal_edge_counter->polled_value();
      shooter_position->pusher_distal.posedge_count =
          distal_edge_counter->positive_interrupt_count();
      shooter_position->pusher_distal.negedge_count =
          distal_edge_counter->negative_interrupt_count();
      shooter_position->pusher_distal.posedge_value =
          shooter_translate(distal_edge_counter->last_positive_encoder_value());
    }

    shooter_position.Send();
  }

 private:
  ::std::unique_ptr<HallEffect> shooter_plunger_;
  ::std::unique_ptr<HallEffect> shooter_latch_;
};

class SensorReader {
 public:
  SensorReader()
      : auto_selector_analog_(new AnalogInput(4)),
        left_encoder_(new Encoder(11, 10, false, Encoder::k2X)),   // E0
        right_encoder_(new Encoder(13, 12, false, Encoder::k2X)),  // E1
        low_left_drive_hall_(new AnalogInput(1)),
        high_left_drive_hall_(new AnalogInput(0)),
        low_right_drive_hall_(new AnalogInput(2)),
        high_right_drive_hall_(new AnalogInput(3)),
        shooter_plunger_(new HallEffect(8)),            // S3
        shooter_latch_(new HallEffect(9)),              // S4
        shooter_distal_(new HallEffect(7)),             // S2
        shooter_proximal_(new HallEffect(6)),           // S1
        shooter_encoder_(new Encoder(14, 15)),          // E2
        claw_top_front_hall_(new HallEffect(4)),        // R2
        claw_top_calibration_hall_(new HallEffect(3)),  // R3
        claw_top_back_hall_(new HallEffect(5)),         // R1
        claw_top_encoder_(new Encoder(17, 16)),         // E3
        // L2  Middle Left hall effect sensor.
        claw_bottom_front_hall_(new HallEffect(1)),
        // L3  Bottom Left hall effect sensor
        claw_bottom_calibration_hall_(new HallEffect(0)),
        // L1  Top Left hall effect sensor
        claw_bottom_back_hall_(new HallEffect(2)),
        claw_bottom_encoder_(new Encoder(21, 20)),  // E5
        run_(true) {
    filter_.SetPeriodNanoSeconds(100000);
    filter_.Add(shooter_plunger_.get());
    filter_.Add(shooter_latch_.get());
    filter_.Add(shooter_distal_.get());
    filter_.Add(shooter_proximal_.get());
    filter_.Add(claw_top_front_hall_.get());
    filter_.Add(claw_top_calibration_hall_.get());
    filter_.Add(claw_top_back_hall_.get());
    filter_.Add(claw_bottom_front_hall_.get());
    filter_.Add(claw_bottom_calibration_hall_.get());
    filter_.Add(claw_bottom_back_hall_.get());
  }

  void operator()() {
    ::aos::SetCurrentThreadName("SensorReader");

    const int kPriority = 30;
    const int kInterruptPriority = 55;

    ::std::array<::std::unique_ptr<HallEffect>, 2> shooter_sensors;
    shooter_sensors[0] = ::std::move(shooter_proximal_);
    shooter_sensors[1] = ::std::move(shooter_distal_);
    ShooterHallSynchronizer shooter(
        kPriority, kInterruptPriority, ::std::move(shooter_encoder_),
        &shooter_sensors, ::std::move(shooter_plunger_),
        ::std::move(shooter_latch_));
    shooter.StartThread();

    ::std::array<::std::unique_ptr<HallEffect>, 3> claw_top_sensors;
    claw_top_sensors[0] = ::std::move(claw_top_front_hall_);
    claw_top_sensors[1] = ::std::move(claw_top_calibration_hall_);
    claw_top_sensors[2] = ::std::move(claw_top_back_hall_);
    HalfClawHallSynchronizer top_claw("top_claw", kPriority, kInterruptPriority,
                                      ::std::move(claw_top_encoder_),
                                      &claw_top_sensors, false);

    ::std::array<::std::unique_ptr<HallEffect>, 3> claw_bottom_sensors;
    claw_bottom_sensors[0] = ::std::move(claw_bottom_front_hall_);
    claw_bottom_sensors[1] = ::std::move(claw_bottom_calibration_hall_);
    claw_bottom_sensors[2] = ::std::move(claw_bottom_back_hall_);
    HalfClawHallSynchronizer bottom_claw(
        "bottom_claw", kPriority, kInterruptPriority,
        ::std::move(claw_bottom_encoder_), &claw_bottom_sensors, true);

    ::aos::SetCurrentThreadRealtimePriority(kPriority);
    while (run_) {
      ::aos::time::PhasedLoopXMS(10, 9000);
      RunIteration();

      auto claw_position =
          control_loops::claw_queue_group.position.MakeMessage();
      bottom_claw.set_position(&claw_position->bottom);
      top_claw.set_position(&claw_position->top);
      while (true) {
        bottom_claw.StartIteration();
        top_claw.StartIteration();

        // Wait more than the amount of time it takes for a digital input change
        // to go from visible to software to having triggered an interrupt.
        ::aos::time::SleepFor(::aos::time::Time::InUS(120));

        if (bottom_claw.TryFinishingIteration() &&
            top_claw.TryFinishingIteration()) {
          break;
        }
      }

      claw_position.Send();
    }
    shooter.Quit();
    top_claw.Quit();
    bottom_claw.Quit();
  }

  void RunIteration() {
    //::aos::time::TimeFreezer time_freezer;
    DriverStation *ds = DriverStation::GetInstance();

    if (ds->IsSysActive()) {
      auto message = ::aos::controls::output_check_received.MakeMessage();
      // TODO(brians): Actually read a pulse value from the roboRIO.
      message->pwm_value = 0;
      message->pulse_length = -1;
      LOG_STRUCT(DEBUG, "received", *message);
      message.Send();
    }

    ::frc971::sensors::auto_mode.MakeWithBuilder()
        .voltage(auto_selector_analog_->GetVoltage())
        .Send();

    // TODO(austin): Calibrate the shifter constants again.
    drivetrain.position.MakeWithBuilder()
        .right_encoder(drivetrain_translate(right_encoder_->GetRaw()))
        .left_encoder(-drivetrain_translate(left_encoder_->GetRaw()))
        .left_shifter_position(
             hall_translate(constants::GetValues().left_drive,
                            low_left_drive_hall_->GetVoltage(),
                            high_left_drive_hall_->GetVoltage()))
        .right_shifter_position(
             hall_translate(constants::GetValues().right_drive,
                            low_right_drive_hall_->GetVoltage(),
                            high_right_drive_hall_->GetVoltage()))
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
  ::std::unique_ptr<AnalogInput> low_left_drive_hall_;
  ::std::unique_ptr<AnalogInput> high_left_drive_hall_;
  ::std::unique_ptr<AnalogInput> low_right_drive_hall_;
  ::std::unique_ptr<AnalogInput> high_right_drive_hall_;

  ::std::unique_ptr<HallEffect> shooter_plunger_;
  ::std::unique_ptr<HallEffect> shooter_latch_;
  ::std::unique_ptr<HallEffect> shooter_distal_;
  ::std::unique_ptr<HallEffect> shooter_proximal_;
  ::std::unique_ptr<Encoder> shooter_encoder_;

  ::std::unique_ptr<HallEffect> claw_top_front_hall_;
  ::std::unique_ptr<HallEffect> claw_top_calibration_hall_;
  ::std::unique_ptr<HallEffect> claw_top_back_hall_;
  ::std::unique_ptr<Encoder> claw_top_encoder_;

  ::std::unique_ptr<HallEffect> claw_bottom_front_hall_;
  ::std::unique_ptr<HallEffect> claw_bottom_calibration_hall_;
  ::std::unique_ptr<HallEffect> claw_bottom_back_hall_;
  ::std::unique_ptr<Encoder> claw_bottom_encoder_;

  ::std::atomic<bool> run_;
  DigitalGlitchFilter filter_;
};

class SolenoidWriter {
 public:
  SolenoidWriter(const ::std::unique_ptr<BufferedPcm> &pcm)
      : pcm_(pcm),
        drivetrain_(".frc971.control_loops.drivetrain.output"),
        shooter_(".frc971.control_loops.shooter_queue_group.output") {}

  void set_drivetrain_left(::std::unique_ptr<BufferedSolenoid> s) {
    drivetrain_left_ = ::std::move(s);
  }

  void set_drivetrain_right(::std::unique_ptr<BufferedSolenoid> s) {
    drivetrain_right_ = ::std::move(s);
  }

  void set_shooter_latch(::std::unique_ptr<BufferedSolenoid> s) {
    shooter_latch_ = ::std::move(s);
  }

  void set_shooter_brake(::std::unique_ptr<BufferedSolenoid> s) {
    shooter_brake_ = ::std::move(s);
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

      {
        shooter_.FetchLatest();
        if (shooter_.get()) {
          LOG_STRUCT(DEBUG, "solenoids", *shooter_);
          shooter_latch_->Set(!shooter_->latch_piston);
          shooter_brake_->Set(!shooter_->brake_piston);
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
  ::std::unique_ptr<BufferedSolenoid> shooter_latch_;
  ::std::unique_ptr<BufferedSolenoid> shooter_brake_;

  ::aos::Queue<::frc971::control_loops::Drivetrain::Output> drivetrain_;
  ::aos::Queue<::frc971::control_loops::ShooterGroup::Output> shooter_;

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

class ShooterWriter : public LoopOutputHandler {
 public:
  void set_shooter_talon(::std::unique_ptr<Talon> t) {
    shooter_talon_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::frc971::control_loops::shooter_queue_group.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::frc971::control_loops::shooter_queue_group.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    shooter_talon_->Set(queue->voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "shooter output too old\n");
    shooter_talon_->Disable();
  }

  ::std::unique_ptr<Talon> shooter_talon_;
};

class ClawWriter : public LoopOutputHandler {
 public:
  void set_top_claw_talon(::std::unique_ptr<Talon> t) {
    top_claw_talon_ = ::std::move(t);
  }

  void set_bottom_claw_talon(::std::unique_ptr<Talon> t) {
    bottom_claw_talon_ = ::std::move(t);
  }

  void set_left_tusk_talon(::std::unique_ptr<Talon> t) {
    left_tusk_talon_ = ::std::move(t);
  }

  void set_right_tusk_talon(::std::unique_ptr<Talon> t) {
    right_tusk_talon_ = ::std::move(t);
  }

  void set_intake1_talon(::std::unique_ptr<Talon> t) {
    intake1_talon_ = ::std::move(t);
  }

  void set_intake2_talon(::std::unique_ptr<Talon> t) {
    intake2_talon_ = ::std::move(t);
  }

 private:
  virtual void Read() override {
    ::frc971::control_loops::claw_queue_group.output.FetchAnother();
  }

  virtual void Write() override {
    auto &queue = ::frc971::control_loops::claw_queue_group.output;
    LOG_STRUCT(DEBUG, "will output", *queue);
    intake1_talon_->Set(queue->intake_voltage / 12.0);
    intake2_talon_->Set(queue->intake_voltage / 12.0);
    bottom_claw_talon_->Set(-queue->bottom_claw_voltage / 12.0);
    top_claw_talon_->Set(queue->top_claw_voltage / 12.0);
    left_tusk_talon_->Set(queue->tusk_voltage / 12.0);
    right_tusk_talon_->Set(-queue->tusk_voltage / 12.0);
  }

  virtual void Stop() override {
    LOG(WARNING, "claw output too old\n");
    intake1_talon_->Disable();
    intake2_talon_->Disable();
    bottom_claw_talon_->Disable();
    top_claw_talon_->Disable();
    left_tusk_talon_->Disable();
    right_tusk_talon_->Disable();
  }

  ::std::unique_ptr<Talon> top_claw_talon_;
  ::std::unique_ptr<Talon> bottom_claw_talon_;
  ::std::unique_ptr<Talon> left_tusk_talon_;
  ::std::unique_ptr<Talon> right_tusk_talon_;
  ::std::unique_ptr<Talon> intake1_talon_;
  ::std::unique_ptr<Talon> intake2_talon_;
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

    ::frc971::wpilib::ClawWriter claw_writer;
    claw_writer.set_top_claw_talon(::std::unique_ptr<Talon>(new Talon(1)));
    claw_writer.set_bottom_claw_talon(::std::unique_ptr<Talon>(new Talon(0)));
    claw_writer.set_left_tusk_talon(::std::unique_ptr<Talon>(new Talon(4)));
    claw_writer.set_right_tusk_talon(::std::unique_ptr<Talon>(new Talon(3)));
    claw_writer.set_intake1_talon(::std::unique_ptr<Talon>(new Talon(7)));
    claw_writer.set_intake2_talon(::std::unique_ptr<Talon>(new Talon(8)));
    ::std::thread claw_writer_thread(::std::ref(claw_writer));

    ::frc971::wpilib::ShooterWriter shooter_writer;
    shooter_writer.set_shooter_talon(::std::unique_ptr<Talon>(new Talon(6)));
    ::std::thread shooter_writer_thread(::std::ref(shooter_writer));

    ::std::unique_ptr<::frc971::wpilib::BufferedPcm> pcm(
        new ::frc971::wpilib::BufferedPcm());
    ::frc971::wpilib::SolenoidWriter solenoid_writer(pcm);
    solenoid_writer.set_drivetrain_left(pcm->MakeSolenoid(6));
    solenoid_writer.set_drivetrain_right(pcm->MakeSolenoid(7));
    solenoid_writer.set_shooter_latch(pcm->MakeSolenoid(5));
    solenoid_writer.set_shooter_brake(pcm->MakeSolenoid(4));
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
    claw_writer.Quit();
    claw_writer_thread.join();
    shooter_writer.Quit();
    shooter_writer_thread.join();
    solenoid_writer.Quit();
    solenoid_thread.join();

    ::aos::Cleanup();
  }
};


START_ROBOT_CLASS(WPILibRobot);
