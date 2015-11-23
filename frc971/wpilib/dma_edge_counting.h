#ifndef FRC971_WPILIB_DMA_EDGE_COUNTING_H_
#define FRC971_WPILIB_DMA_EDGE_COUNTING_H_

#include <memory>
#include <vector>

#include "aos/common/macros.h"

#include "DigitalInput.h"
#include "Encoder.h"
#include "AnalogInput.h"
#include "Utility.h"
#include "dma.h"
#undef ERROR

namespace frc971 {
namespace wpilib {

// Generic interface for classes that do something with DMA samples and also
// poll current sensor values.
class DMASampleHandlerInterface {
 public:
  virtual ~DMASampleHandlerInterface() {}

  // Updates values based on a new DMA sample.
  virtual void UpdateFromSample(const DMASample &sample) = 0;

  // Polls the current values and saves them for later reference.
  virtual void UpdatePolledValue() = 0;

  // Fills in the "polled" values from sample.
  // This is only called when a DMA event happens right as we're polling values.
  virtual void PollFromSample(const DMASample &sample) = 0;

  // Adds readings and triggers appropriate to this object to dma.
  virtual void AddToDMA(DMA *dma) = 0;
};

// Counts edges on a sensor using DMA data and latches encoder values
// corresponding to those edges.
class DMAEdgeCounter : public DMASampleHandlerInterface {
 public:
  DMAEdgeCounter(Encoder *encoder, DigitalInput *input)
      : encoder_(encoder), input_(input) {}

  int positive_count() const { return pos_edge_count_; }
  int negative_count() const { return neg_edge_count_; }
  int last_positive_encoder_value() const { return pos_last_encoder_; }
  int last_negative_encoder_value() const { return neg_last_encoder_; }

  // Returns the value of the sensor in the last-read DMA sample.
  bool last_value() const { return ExtractValue(prev_sample_); }
  // Returns the most recent polled value of the sensor.
  bool polled_value() const { return polled_value_; }
  // Returns the most recent polled reading from the encoder.
  int polled_encoder() const { return polled_encoder_; }

 private:
  void UpdateFromSample(const DMASample &sample) override;
  void UpdatePolledValue() override {
    polled_value_ = input_->Get();
    polled_encoder_ = encoder_->GetRaw();
  }
  void PollFromSample(const DMASample &sample) override {
    polled_value_ = ExtractValue(sample);
    polled_encoder_ = sample.GetRaw(encoder_);
  }
  void AddToDMA(DMA *dma) override {
    dma->Add(encoder_);
    dma->Add(input_);
    dma->SetExternalTrigger(input_, true, true);
  }

  bool ExtractValue(const DMASample &sample) const;

  Encoder *const encoder_;
  DigitalInput *const input_;

  // The last DMA reading we got.
  DMASample prev_sample_;
  // Whether or not we actually have anything in prev_sample_.
  bool have_prev_sample_ = false;

  // Values related to the positive edge.
  int pos_edge_count_ = 0;
  double pos_edge_time_ = 0.0;
  int pos_last_encoder_ = 0;

  // Values related to the negative edge.
  int neg_edge_count_ = 0;
  double neg_edge_time_ = 0.0;
  int neg_last_encoder_ = 0;

  bool polled_value_ = false;
  int polled_encoder_ = 0;

  DISALLOW_COPY_AND_ASSIGN(DMAEdgeCounter);
};

// Reads a hall effect in sync with DMA samples.
class DMADigitalReader : public DMASampleHandlerInterface {
 public:
  DMADigitalReader(DigitalInput *input) : input_(input) {}

  bool value() const { return value_; }

 private:
  void UpdateFromSample(const DMASample & /*sample*/) override {}
  void UpdatePolledValue() override { value_ = input_->Get(); }
  void PollFromSample(const DMASample &sample) override {
    value_ = sample.Get(input_);
  }
  void AddToDMA(DMA *dma) override {
    dma->Add(input_);
  }

  DigitalInput *const input_;

  bool value_;

  DISALLOW_COPY_AND_ASSIGN(DMADigitalReader);
};

// Reads an analog sensor in sync with DMA samples.
class DMAAnalogReader : public DMASampleHandlerInterface {
 public:
  DMAAnalogReader(AnalogInput *input) : input_(input) {}

  double value() const { return value_; }

 private:
  void UpdateFromSample(const DMASample & /*sample*/) override {}
  void UpdatePolledValue() override { value_ = input_->GetVoltage(); }
  void PollFromSample(const DMASample &sample) override {
    value_ = sample.GetVoltage(input_);
  }
  void AddToDMA(DMA *dma) override {
    dma->Add(input_);
  }

  AnalogInput *const input_;

  double value_;

  DISALLOW_COPY_AND_ASSIGN(DMAAnalogReader);
};

// This class handles updating the sampled data on multiple
// DMASampleHandlerInterfaces. The caller should create an instance and then
// periodically call RunIteration, retrieving whatever data from the
// DMASampleHandlerInterfaces after each iteration.
class DMASynchronizer {
 public:
  DMASynchronizer(::std::unique_ptr<DMA> dma) : dma_(::std::move(dma)) {}

  // Adds a new handler to this object. This method must not be called after
  // Start().
  void Add(DMASampleHandlerInterface *handler) {
    handler->AddToDMA(dma_.get());
    handlers_.emplace_back(handler);
  }

  // Actually starts watching for DMA samples.
  // Add may not be called any more after this.
  void Start() {
    dma_->Start(1024);
  }

  // Updates all sensor values.
  void RunIteration() {
    SampleSensors();
    CheckDMA();
  }

 private:
  // Reads the state of all the sensors and records it as the polled values of
  // all the inputs.
  void SampleSensors() {
    sample_time_ = GetFPGATime();
    for (auto &c : handlers_) {
      c->UpdatePolledValue();
    }
  }

  // Gets called by the DMA handler to update edge counts.
  void CheckDMA();

  const ::std::unique_ptr<DMA> dma_;
  ::std::vector<DMASampleHandlerInterface *> handlers_;

  // The time at which we most recently read the sensor values.
  uint32_t sample_time_ = 0.0;

  DISALLOW_COPY_AND_ASSIGN(DMASynchronizer);
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_DMA_EDGE_COUNTING_H_
