#include <stdio.h>
#include <string.h>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <inttypes.h>

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
#include "aos/common/network/team_number.h"
#include "aos/linux_code/init.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/shooter/shooter.q.h"
#include "frc971/constants.h"
#include "frc971/queues/other_sensors.q.h"
#include "frc971/queues/to_log.q.h"

#include <WPILib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::aos::util::SimpleLogInterval;
using ::frc971::control_loops::drivetrain;
using ::frc971::sensors::other_sensors;
using ::frc971::sensors::gyro_reading;
using ::aos::util::WrappingCounter;

namespace frc971 {
namespace output {

void SetThreadRealtimePriority(int priority) {
  struct sched_param param;
  param.sched_priority = priority;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    PLOG(FATAL, "sched_setscheduler failed");
  }
}

class priority_mutex {
 public:
  typedef pthread_mutex_t *native_handle_type;

  // TODO(austin): Write a test case for the mutex, and make the constructor
  // constexpr.
  priority_mutex() {
    pthread_mutexattr_t attr;
#ifdef NDEBUG
#error "Won't let perror be no-op ed"
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

class HallEffect : public DigitalInput {
 public:
  HallEffect(int index) : DigitalInput(index) {}
  bool GetHall() { return !Get(); }
};

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
    SetThreadRealtimePriority(priority_);

    input_->RequestInterrupts();
    input_->SetUpSourceEdge(true, true);

    {
      ::std::unique_lock<priority_mutex> mutex_guard(*mutex_);
      current_value_ = input_->GetHall();
    }

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

// This class will synchronize sampling edges on a bunch of DigitalInputs with
// the periodic poll.
//
// The data is provided to subclasses by calling SaveState when the state is
// consistent and ready.
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
    SetThreadRealtimePriority(priority_);
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

double gyro_translate(int64_t in) {
  return in / 16.0 / 1000.0 / (180.0 / M_PI);
}

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
        left_encoder_(new Encoder(10, 11, false, Encoder::k2X)),   // E0
        right_encoder_(new Encoder(12, 13, false, Encoder::k2X)),  // E1
        low_left_drive_hall_(new AnalogInput(2)),
        high_left_drive_hall_(new AnalogInput(3)),
        low_right_drive_hall_(new AnalogInput(1)),
        high_right_drive_hall_(new AnalogInput(0)),
        shooter_plunger_(new HallEffect(1)),            // S3
        shooter_latch_(new HallEffect(0)),              // S4
        shooter_distal_(new HallEffect(2)),             // S2
        shooter_proximal_(new HallEffect(3)),           // S1
        shooter_encoder_(new Encoder(15, 14)),          // E2
        claw_top_front_hall_(new HallEffect(5)),        // R2
        claw_top_calibration_hall_(new HallEffect(6)),  // R3
        claw_top_back_hall_(new HallEffect(4)),         // R2
        claw_top_encoder_(new Encoder(16, 17)),         // E3
        // L2  Middle Left hall effect sensor.
        claw_bottom_front_hall_(new HallEffect(8)),
        // L3  Bottom Left hall effect sensor
        claw_bottom_calibration_hall_(new HallEffect(9)),
        // L1  Top Left hall effect sensor
        claw_bottom_back_hall_(new HallEffect(7)),
        claw_bottom_encoder_(new Encoder(18, 19)),  // E5
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
    printf("Filtering all hall effect sensors with a %" PRIu64
           " nanosecond window\n",
           filter_.GetPeriodNanoSeconds());
  }

  void operator()() {
    const int kPriority = 30;
    const int kInterruptPriority = 55;
    SetThreadRealtimePriority(kPriority);

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

    bool bad_gyro = true;
    // TODO(brians): Switch to LogInterval for these things.
    /*
    if (data->uninitialized_gyro) {
      LOG(DEBUG, "uninitialized gyro\n");
      bad_gyro = true;
    } else if (data->zeroing_gyro) {
      LOG(DEBUG, "zeroing gyro\n");
      bad_gyro = true;
    } else if (data->bad_gyro) {
      LOG(ERROR, "bad gyro\n");
      bad_gyro = true;
    } else if (data->old_gyro_reading) {
      LOG(WARNING, "old/bad gyro reading\n");
      bad_gyro = true;
    } else {
      bad_gyro = false;
    }
    */

    if (!bad_gyro) {
      // TODO(austin): Read the gyro.
      gyro_reading.MakeWithBuilder().angle(0).Send();
    }

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

    // Signal that we are allive.
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

class MotorWriter {
 public:
  MotorWriter()
      : right_drivetrain_talon_(new Talon(2)),
        left_drivetrain_talon_(new Talon(5)),
        shooter_talon_(new Talon(6)),
        top_claw_talon_(new Talon(1)),
        bottom_claw_talon_(new Talon(0)),
        left_tusk_talon_(new Talon(4)),
        right_tusk_talon_(new Talon(3)),
        intake1_talon_(new Talon(7)),
        intake2_talon_(new Talon(8)),
        left_shifter_(new Solenoid(6)),
        right_shifter_(new Solenoid(7)),
        shooter_latch_(new Solenoid(5)),
        shooter_brake_(new Solenoid(4)),
        compressor_(new Compressor()) {
    compressor_->SetClosedLoopControl(true);
    // right_drivetrain_talon_->EnableDeadbandElimination(true);
    // left_drivetrain_talon_->EnableDeadbandElimination(true);
    // shooter_talon_->EnableDeadbandElimination(true);
    // top_claw_talon_->EnableDeadbandElimination(true);
    // bottom_claw_talon_->EnableDeadbandElimination(true);
    // left_tusk_talon_->EnableDeadbandElimination(true);
    // right_tusk_talon_->EnableDeadbandElimination(true);
    // intake1_talon_->EnableDeadbandElimination(true);
    // intake2_talon_->EnableDeadbandElimination(true);
  }

  // Maximum age of an output packet before the motors get zeroed instead.
  static const int kOutputMaxAgeMS = 20;
  static constexpr ::aos::time::Time kOldLogInterval =
      ::aos::time::Time::InSeconds(0.5);

  void Run() {
    //::aos::time::Time::EnableMockTime();
    while (true) {
      //::aos::time::Time::UpdateMockTime();
      // 200 hz loop
      ::aos::time::PhasedLoopXMS(5, 1000);
      //::aos::time::Time::UpdateMockTime();

      no_robot_state_.Print();
      fake_robot_state_.Print();
      sending_failed_.Print();

      RunIteration();
    }
  }

  virtual void RunIteration() {
    ::aos::robot_state.FetchLatest();
    if (!::aos::robot_state.get()) {
      LOG_INTERVAL(no_robot_state_);
      return;
    }
    if (::aos::robot_state->fake) {
      LOG_INTERVAL(fake_robot_state_);
      return;
    }

    // TODO(austin): Write the motor values out when they change!  One thread
    // per queue.
    // TODO(austin): Figure out how to synchronize everything to the PWM update
    // rate, or get the pulse to go out clocked off of this code.  That would be
    // awesome.
    {
      static auto &drivetrain = ::frc971::control_loops::drivetrain.output;
      drivetrain.FetchLatest();
      if (drivetrain.IsNewerThanMS(kOutputMaxAgeMS)) {
        LOG_STRUCT(DEBUG, "will output", *drivetrain);
        left_drivetrain_talon_->Set(-drivetrain->left_voltage / 12.0);
        right_drivetrain_talon_->Set(drivetrain->right_voltage / 12.0);
        left_shifter_->Set(drivetrain->left_high);
        right_shifter_->Set(drivetrain->right_high);
      } else {
        left_drivetrain_talon_->Disable();
        right_drivetrain_talon_->Disable();
        LOG_INTERVAL(drivetrain_old_);
      }
      drivetrain_old_.Print();
    }

    {
      static auto &shooter =
          ::frc971::control_loops::shooter_queue_group.output;
      shooter.FetchLatest();
      if (shooter.IsNewerThanMS(kOutputMaxAgeMS)) {
        LOG_STRUCT(DEBUG, "will output", *shooter);
        shooter_talon_->Set(shooter->voltage / 12.0);
        shooter_latch_->Set(!shooter->latch_piston);
        shooter_brake_->Set(!shooter->brake_piston);
      } else {
        shooter_talon_->Disable();
        shooter_brake_->Set(false);  // engage the brake
        LOG_INTERVAL(shooter_old_);
      }
      shooter_old_.Print();
    }

    {
      static auto &claw = ::frc971::control_loops::claw_queue_group.output;
      claw.FetchLatest();
      if (claw.IsNewerThanMS(kOutputMaxAgeMS)) {
        LOG_STRUCT(DEBUG, "will output", *claw);
        intake1_talon_->Set(claw->intake_voltage / 12.0);
        intake2_talon_->Set(claw->intake_voltage / 12.0);
        bottom_claw_talon_->Set(-claw->bottom_claw_voltage / 12.0);
        top_claw_talon_->Set(claw->top_claw_voltage / 12.0);
        left_tusk_talon_->Set(claw->tusk_voltage / 12.0);
        right_tusk_talon_->Set(-claw->tusk_voltage / 12.0);
      } else {
        intake1_talon_->Disable();
        intake2_talon_->Disable();
        bottom_claw_talon_->Disable();
        top_claw_talon_->Disable();
        left_tusk_talon_->Disable();
        right_tusk_talon_->Disable();
        LOG_INTERVAL(claw_old_);
      }
      claw_old_.Print();
    }
  }

  SimpleLogInterval drivetrain_old_ =
      SimpleLogInterval(kOldLogInterval, WARNING, "drivetrain too old");
  SimpleLogInterval shooter_old_ =
      SimpleLogInterval(kOldLogInterval, WARNING, "shooter too old");
  SimpleLogInterval claw_old_ =
      SimpleLogInterval(kOldLogInterval, WARNING, "claw too old");

  ::std::unique_ptr<Talon> right_drivetrain_talon_;
  ::std::unique_ptr<Talon> left_drivetrain_talon_;
  ::std::unique_ptr<Talon> shooter_talon_;
  ::std::unique_ptr<Talon> top_claw_talon_;
  ::std::unique_ptr<Talon> bottom_claw_talon_;
  ::std::unique_ptr<Talon> left_tusk_talon_;
  ::std::unique_ptr<Talon> right_tusk_talon_;
  ::std::unique_ptr<Talon> intake1_talon_;
  ::std::unique_ptr<Talon> intake2_talon_;

  ::std::unique_ptr<Solenoid> left_shifter_;
  ::std::unique_ptr<Solenoid> right_shifter_;
  ::std::unique_ptr<Solenoid> shooter_latch_;
  ::std::unique_ptr<Solenoid> shooter_brake_;

  ::std::unique_ptr<Compressor> compressor_;

  ::aos::util::SimpleLogInterval no_robot_state_ =
      ::aos::util::SimpleLogInterval(::aos::time::Time::InSeconds(0.5), INFO,
                                     "no robot state -> not outputting");
  ::aos::util::SimpleLogInterval fake_robot_state_ =
      ::aos::util::SimpleLogInterval(::aos::time::Time::InSeconds(0.5), DEBUG,
                                     "fake robot state -> not outputting");
  ::aos::util::SimpleLogInterval sending_failed_ =
      ::aos::util::SimpleLogInterval(::aos::time::Time::InSeconds(0.1), WARNING,
                                     "sending outputs failed");
};

constexpr ::aos::time::Time MotorWriter::kOldLogInterval;

class JoystickSender {
 public:
  JoystickSender() : run_(true) {}

  void operator()() {
    DriverStation *ds = DriverStation::GetInstance();
    SetThreadRealtimePriority(29);
    uint16_t team_id = ::aos::network::GetTeamNumber();

    while (run_) {
      ds->WaitForData();
      auto new_state = ::aos::robot_state.MakeMessage();

      new_state->test_mode = ds->IsAutonomous();
      new_state->fms_attached = ds->IsFMSAttached();
      new_state->enabled = ds->IsEnabled();
      new_state->autonomous = ds->IsAutonomous();
      new_state->team_id = team_id;
      new_state->fake = false;

      for (int i = 0; i < 4; ++i) {
        new_state->joysticks[i].buttons = ds->GetStickButtons(i);
        for (int j = 0; j < 4; ++j) {
          new_state->joysticks[i].axis[j] = ds->GetStickAxis(i, j + 1);
        }
      }
      LOG_STRUCT(DEBUG, "robot_state", *new_state);

      if (!new_state.Send()) {
        LOG(WARNING, "sending robot_state failed\n");
      }
    }
  }

  void Quit() { run_ = false; }

 private:
  ::std::atomic<bool> run_;
};
}  // namespace output
}  // namespace frc971

class WPILibRobot : public RobotBase {
 public:
  virtual void StartCompetition() {
    ::aos::Init();
    ::frc971::output::MotorWriter writer;
    ::frc971::output::SensorReader reader;
    ::std::thread reader_thread(::std::ref(reader));
    ::frc971::output::JoystickSender joystick_sender;
    ::std::thread joystick_thread(::std::ref(joystick_sender));
    writer.Run();
    LOG(ERROR, "Exiting WPILibRobot\n");
    reader.Quit();
    reader_thread.join();
    joystick_sender.Quit();
    joystick_thread.join();
    ::aos::Cleanup();
  }
};


START_ROBOT_CLASS(WPILibRobot);
