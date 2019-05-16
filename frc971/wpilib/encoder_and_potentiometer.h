#ifndef FRC971_ENCODER_AND_POTENTIOMETER_H_
#define FRC971_ENCODER_AND_POTENTIOMETER_H_

#include <atomic>
#include <cmath>
#include <thread>

#include "aos/macros.h"
#include "aos/mutex/mutex.h"
#include "aos/time/time.h"

#include "frc971/wpilib/ahal/AnalogInput.h"
#include "frc971/wpilib/ahal/Counter.h"
#include "frc971/wpilib/ahal/DigitalSource.h"
#include "frc971/wpilib/ahal/Encoder.h"

#include "frc971/wpilib/dma.h"
#include "frc971/wpilib/dma_edge_counting.h"

namespace frc971 {
namespace wpilib {

// Latches values from an encoder and potentiometer on positive edges from
// another input using an interrupt.
class InterruptEncoderAndPotentiometer {
 public:
  // priority is the priority the thread will run at.
  InterruptEncoderAndPotentiometer(int priority) : priority_(priority) {}

  // Starts the thread running so it can receive interrupts.
  void Start();

  // Tells the thread to stop running and then waits for it to finish.
  void Stop() {
    run_ = false;
    thread_.join();
  }

  // Loops until Stop() is called, reading interrupts.
  // Designed to be called by ::std::thread internally.
  void operator()();

  // Returns the mutex which must be held while calling index_posedge_count(),
  // last_encoder_value(), and last_potentiometer_voltage().
  // Holding this mutex will increase the handling latency.
  ::aos::Mutex *mutex() { return &mutex_; }

  void set_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    encoder_ = ::std::move(encoder);
  }
  frc::Encoder *encoder() const { return encoder_.get(); }

  void set_index(::std::unique_ptr<frc::DigitalSource> index) {
    index_ = ::std::move(index);
  }
  frc::DigitalSource *index() const { return index_.get(); }

  void set_potentiometer(::std::unique_ptr<frc::AnalogInput> potentiometer) {
    potentiometer_ = ::std::move(potentiometer);
  }
  frc::AnalogInput *potentiometer() const { return potentiometer_.get(); }

  // Returns the number of poseges that have happened on the index input.
  // mutex() must be held while calling this.
  uint32_t index_posedge_count() const { return index_posedge_count_; }
  // Returns the value of the encoder at the last index posedge.
  // mutex() must be held while calling this.
  int32_t last_encoder_value() const { return last_encoder_value_; }
  // Returns the voltage of the potentiometer at the last index posedge.
  // mutex() must be held while calling this.
  float last_potentiometer_voltage() const {
    return last_potentiometer_voltage_;
  }

 private:
  ::std::unique_ptr<frc::Encoder> encoder_;
  ::std::unique_ptr<frc::DigitalSource> index_;
  ::std::unique_ptr<frc::AnalogInput> potentiometer_;

  int32_t last_encoder_value_{0};
  float last_potentiometer_voltage_{0.0f};
  uint32_t index_posedge_count_{0};

  ::aos::Mutex mutex_;

  const int priority_;

  ::std::atomic<bool> run_{true};
  ::std::thread thread_;

  DISALLOW_COPY_AND_ASSIGN(InterruptEncoderAndPotentiometer);
};

// Latches values from an encoder on positive edges from another input using
// DMA.
class DMAEncoder : public DMASampleHandlerInterface {
 public:
  DMAEncoder() {}

  void set_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    encoder_ = ::std::move(encoder);
  }
  frc::Encoder *encoder() const { return encoder_.get(); }

  void set_index(::std::unique_ptr<frc::DigitalSource> index) {
    index_ = ::std::move(index);
  }
  frc::DigitalSource *index() const { return index_.get(); }

  // Returns the most recent polled value of the encoder.
  uint32_t polled_encoder_value() const { return polled_encoder_value_; }

  // Returns the number of poseges that have happened on the index input.
  uint32_t index_posedge_count() const { return index_posedge_count_; }
  // Returns the value of the encoder at the last index posedge.
  int32_t last_encoder_value() const { return last_encoder_value_; }

  void UpdateFromSample(const DMASample &sample) override {
    DoUpdateFromSample(sample);
  }

  void PollFromSample(const DMASample &sample) override {
    polled_encoder_value_ = sample.GetRaw(encoder_.get());
  }

  void UpdatePolledValue() override {
    polled_encoder_value_ = encoder_->GetRaw();
  }

  void AddToDMA(DMA *dma) override {
    dma->Add(encoder_.get());
    dma->Add(index_.get());
    dma->SetExternalTrigger(index_.get(), true, true);
  }

 protected:
  // The same as UpdateFromSample except also returns true if this sample is a
  // new edge on the index.
  bool DoUpdateFromSample(const DMASample &sample);

 private:
  ::std::unique_ptr<frc::Encoder> encoder_;
  ::std::unique_ptr<frc::DigitalSource> index_;

  int32_t polled_encoder_value_ = 0;

  int32_t last_encoder_value_ = 0;

  uint32_t index_posedge_count_ = 0;

  // Whether or not it was triggered in the last sample.
  bool index_last_value_ = false;

  DISALLOW_COPY_AND_ASSIGN(DMAEncoder);
};

// Latches values from an encoder and potentiometer on positive edges from
// another input using DMA.
class DMAEncoderAndPotentiometer : public DMAEncoder {
 public:
  DMAEncoderAndPotentiometer() {}

  void set_potentiometer(::std::unique_ptr<frc::AnalogInput> potentiometer) {
    potentiometer_ = ::std::move(potentiometer);
  }
  frc::AnalogInput *potentiometer() const { return potentiometer_.get(); }

  // Returns the most recent polled voltage of the potentiometer.
  float polled_potentiometer_voltage() const {
    return polled_potentiometer_voltage_;
  }

  // Returns the voltage of the potentiometer at the last index posedge.
  float last_potentiometer_voltage() const {
    return last_potentiometer_voltage_;
  }

  void UpdateFromSample(const DMASample &sample) override {
    if (DMAEncoder::DoUpdateFromSample(sample)) {
      last_potentiometer_voltage_ = sample.GetVoltage(potentiometer_.get());
    }
  }

  void PollFromSample(const DMASample &sample) override {
    polled_potentiometer_voltage_ = sample.GetVoltage(potentiometer_.get());
    DMAEncoder::PollFromSample(sample);
  }

  void UpdatePolledValue() override {
    polled_potentiometer_voltage_ = potentiometer_->GetVoltage();
    DMAEncoder::UpdatePolledValue();
  }

  void AddToDMA(DMA *dma) override {
    dma->Add(potentiometer_.get());
    DMAEncoder::AddToDMA(dma);
  }

 private:
  ::std::unique_ptr<frc::AnalogInput> potentiometer_;

  float polled_potentiometer_voltage_ = 0.0f;

  float last_potentiometer_voltage_ = 0.0f;

  DISALLOW_COPY_AND_ASSIGN(DMAEncoderAndPotentiometer);
};

// Class to read duty cycle of an input.  This is tuned for the CTRE encoder's
// absolute position output.
class DutyCycleReader {
 public:
  // Configure the reader to use the provided digital input.
  void set_input(::std::unique_ptr<::frc::DigitalInput> input) {
    high_counter_.reset(new ::frc::Counter(input.get()));
    high_counter_->SetMaxPeriod(kMaxPeriod);
    high_counter_->SetSemiPeriodMode(true);

    period_length_counter_.reset(new ::frc::Counter(input.get()));
    period_length_counter_->SetMaxPeriod(kMaxPeriod);
    period_length_counter_->SetUpSourceEdge(true, false);

    input_ = ::std::move(input);
  }

  // Returns the last duty cycle or nan if the signal is stale.
  double Read() const {
    const double high_time = high_counter_->GetPeriod();
    const double period_length = period_length_counter_->GetPeriod();
    if (!::std::isfinite(high_time) || !::std::isfinite(period_length)) {
      return ::std::numeric_limits<double>::quiet_NaN();
    }
    return high_time / period_length;
  }

 private:
  static constexpr ::std::chrono::nanoseconds kNominalPeriod =
      ::std::chrono::microseconds(4096);
  static constexpr double kMaxPeriod =
      ::aos::time::DurationInSeconds(kNominalPeriod * 2);

  ::std::unique_ptr<::frc::Counter> high_counter_, period_length_counter_;
  ::std::unique_ptr<::frc::DigitalInput> input_;
};

// Class to hold a CTRE encoder with absolute angle pwm and potentiometer pair.
class AbsoluteEncoderAndPotentiometer {
 public:
  void set_absolute_pwm(::std::unique_ptr<frc::DigitalInput> input) {
    duty_cycle_.set_input(::std::move(input));
  }

  void set_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    encoder_ = ::std::move(encoder);
  }

  void set_potentiometer(::std::unique_ptr<frc::AnalogInput> potentiometer) {
    potentiometer_ = ::std::move(potentiometer);
  }

  double ReadAbsoluteEncoder() const { return duty_cycle_.Read(); }
  int32_t ReadRelativeEncoder() const { return encoder_->GetRaw(); }
  double ReadPotentiometerVoltage() const {
    return potentiometer_->GetVoltage();
  }

 private:
  DutyCycleReader duty_cycle_;
  ::std::unique_ptr<frc::Encoder> encoder_;
  ::std::unique_ptr<frc::AnalogInput> potentiometer_;
};

class AbsoluteEncoder {
 public:
  void set_absolute_pwm(::std::unique_ptr<frc::DigitalInput> input) {
    duty_cycle_.set_input(::std::move(input));
  }

  void set_encoder(::std::unique_ptr<frc::Encoder> encoder) {
    encoder_ = ::std::move(encoder);
  }

  double ReadAbsoluteEncoder() const { return duty_cycle_.Read(); }
  int32_t ReadRelativeEncoder() const { return encoder_->GetRaw(); }

 private:
  DutyCycleReader duty_cycle_;
  ::std::unique_ptr<frc::Encoder> encoder_;
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_ENCODER_AND_POTENTIOMETER_H_
