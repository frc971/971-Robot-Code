#ifndef FRC971_WPILIB_INTERRUPT_EDGE_COUNTING_H_
#define FRC971_WPILIB_INTERRUPT_EDGE_COUNTING_H_

#include <memory>
#include <atomic>
#include <thread>
#include <vector>

#include "aos/common/stl_mutex.h"
#include "aos/common/macros.h"

#include "DigitalInput.h"
#include "Encoder.h"
#include "AnalogInput.h"
#include "Utility.h"

namespace frc971 {
namespace wpilib {

class InterruptSynchronizer;

// Handles interrupts arriving from a single source.
//
// Instances of subclasses should be passed to InterruptSynchronizer::Add to use
// them. All methods which are called with the lock held should avoid taking too
// long because they directly contribute to interrupt-handling latency for all
// InterruptHandlers in the same InterruptSynchronizer.
//
// Each instance handles any important sensor reading which must happen right
// after an interrupt triggers from a single source. It is also important that
// some sensors are sampled after any pending interrupts have been processed.
// This is handled using a per-InterruptSynchronizer mutex. Each
// InterruptHandler records that it has received an interrupt, locks the mutex,
// and then updates a "shadow" state. The InterruptSynchronizer then triggers
// making this "shadow" state visible after making sure no more interrupts have
// arrived while holding the mutex.
class InterruptHandler {
 public:
  virtual ~InterruptHandler() {}

  // Stops the thread which actually does the sampling and waits for it to
  // finish.
  void Quit() {
    run_ = false;
    thread_.join();
  }

  // Starts the thread running.
  // set_priority and set_mutex must be called first.
  void Start() {
    CHECK_NE(nullptr, mutex_);
    CHECK_NE(0, priority_);
    thread_ = ::std::thread(::std::ref(*this));
  }

  // Polls the current values and saves them to the "shadow" output.
  // Called while the lock is held.
  virtual void GatherPolledValue() = 0;

  // Actually outputs the "shadow" state collected during the most recent
  // GatherPolledValue.
  // Called while the lock is held.
  virtual void CommitValue() = 0;

  // Saves the current interrupt count to be compared when
  // interrupt_count_changed() is called.
  void save_interrupt_count() { saved_interrupt_count_ = interrupt_count_; }
  // Returns whether or not the interrupt count has changed since
  // save_interrupt_count() was last called.
  bool interrupt_count_changed() const {
    return saved_interrupt_count_ != interrupt_count_;
  }

  // Sets the priority the thread will run at.
  // This must be called before Start.
  void set_priority(int priority) { priority_ = priority; }

  // Sets the mutex to use for synchronizing readings.
  // This must be called before Start.
  void set_mutex(::aos::stl_mutex *mutex) { mutex_ = mutex; }

  // Waits for interrupts, locks the mutex, and updates the internal state.
  // Should only be called by the (internal) ::std::thread.
  virtual void operator()() = 0;

 protected:
  // Indicates that another interrupt has been received (not handled yet).
  void interrupt_received() { ++interrupt_count_; }

  int priority() const { return priority_; }

  ::aos::stl_mutex *mutex() { return mutex_; }

  // Returns true if the thread should continue running.
  bool should_run() const { return run_; }

 private:
  ::std::atomic<int> interrupt_count_{0};
  int saved_interrupt_count_;

  ::std::atomic<bool> run_{true};
  ::std::thread thread_;

  int priority_ = 0;
  ::aos::stl_mutex *mutex_ = nullptr;
};

// Latches the value of an encoder on rising and falling edges of a digital
// input.
class EdgeCounter : public InterruptHandler {
 public:
  EdgeCounter(Encoder *encoder, DigitalInput *input)
      : encoder_(encoder), input_(input) {}

  // Returns the current interrupt edge counts and encoder values.
  int positive_interrupt_count() const {
    return output_.positive_interrupt_count;
  }
  int negative_interrupt_count() const {
    return output_.negative_interrupt_count;
  }
  int32_t last_positive_encoder_value() const {
    return output_.last_positive_encoder_value;
  }
  int32_t last_negative_encoder_value() const {
    return output_.last_negative_encoder_value;
  }
  // Returns the current polled value.
  bool polled_value() const { return output_.polled_value; }

 private:
  struct OutputValues {
    bool polled_value = false;
    int positive_interrupt_count = 0, negative_interrupt_count = 0;
    int32_t last_positive_encoder_value = 0, last_negative_encoder_value = 0;
  };

  void GatherPolledValue() override;
  void CommitValue() override { output_ = shadow_values_; }
  void operator()() override;

  Encoder *encoder_;
  DigitalInput *input_;

  // The following variables represent the current "shadow" state.
  bool current_value_ = false;
  bool last_miss_match_ = true;
  OutputValues shadow_values_;

  // The actual output values.
  OutputValues output_;

  DISALLOW_COPY_AND_ASSIGN(EdgeCounter);
};

// Synchronizes reading an encoder with interrupt handling.
class InterruptSynchronizedEncoder : public InterruptHandler {
 public:
  InterruptSynchronizedEncoder(Encoder *encoder) : encoder_(encoder) {}

  int32_t get() const { return output_; }

 private:
  void GatherPolledValue() override { shadow_ = encoder_->GetRaw(); }
  void CommitValue() override { output_ = shadow_; }
  void operator()() override {}

  Encoder *const encoder_;

  int32_t shadow_, output_;
};

// Synchronizes interrupts with poll-based sampling on multiple
// InterruptHandlers.
//
// See InterruptHandler for an overview of the logic.
//
// Usage is to create an instance, call Add 1 or more times, call Start, and
// then call RunIteration during normal sensor sampling. After RunIteration
// returns, the output values from the various InterruptHandlers can be
// retrieved.
class InterruptSynchronizer {
 public:
  InterruptSynchronizer(int interrupt_priority)
      : interrupt_priority_(interrupt_priority) {}

  void Add(InterruptHandler *handler) {
    handler->set_mutex(&mutex_);
    handler->set_priority(interrupt_priority_);
    handlers_.emplace_back(handler);
  }

  void Start() {
    for (auto &c : handlers_) {
      c->Start();
    }
  }

  // Updates all of the counts and makes sure everything is synchronized.
  // IMPORTANT: This will usually only take 120uS but WILL occasionally take
  // longer, so be careful about letting that jitter get into control loops.
  void RunIteration();

  // Asks all of the InterruptHandlers to stop and waits until they have done
  // so.
  void Quit() {
    for (auto &c : handlers_) {
      c->Quit();
    }
  }

 private:
  // Starts a sampling iteration.  See RunIteration for usage.
  // Returns true if we are ready to go or false if we already need to retry.
  bool TryStartIteration();

  // Attempts to finish a sampling iteration.  See RunIteration for usage.
  // Returns true if the iteration succeeded, and false otherwise.
  bool TryFinishingIteration();

  const int interrupt_priority_;

  // The mutex used to synchronize all the sampling.
  ::aos::stl_mutex mutex_;

  ::std::vector<InterruptHandler *> handlers_;

  DISALLOW_COPY_AND_ASSIGN(InterruptSynchronizer);
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_INTERRUPT_EDGE_COUNTING_H_
