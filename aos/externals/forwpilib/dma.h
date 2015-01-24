#ifndef _DMA_H_
#define _DMA_H_

#include <stdint.h>

#include <array>
#include <memory>

#include "ChipObject.h"
#include "DigitalSource.h"
#include "Encoder.h"

class DMA;

class DMASample {
 public:
  DMASample() {}

  // Returns the FPGA timestamp of the sample.
  double GetTimestamp() const;

  // All Get methods either return the requested value, or set the Error.

  // Returns the value of the digital input in the sample.
  bool Get(DigitalSource *input) const;
  // Returns the raw value of the encoder in the sample.
  int32_t GetRaw(Encoder *input) const;
  // Returns the {1, 2, or 4} X scaled value of the encoder in the sample.
  int32_t Get(Encoder *input) const;

 private:
  friend DMA;

  // Returns the offset of the sample type in the buffer, or -1 if it isn't in
  // the sample.
  ssize_t offset(int index) const;

  // TODO(austin): This should be re-used from WPILib...  Once I merge this back
  // into WPILib.

  DMA *dma_;
  uint32_t read_buffer_[64];
};

class DMA : public ErrorBase {
 public:
  DMA();
  virtual ~DMA();

  // Sets whether or not DMA is paused.
  void SetPause(bool pause);

  // Sets the number of triggers that need to occur before a sample is saved.
  void SetRate(uint32_t cycles);

  // Adds the input signal to the state to snapshot on the trigger event.
  // Call Add() and SetExternalTrigger() before Start().
  void Add(Encoder *encoder);
  void Add(DigitalSource *input);

  // Configures DMA to trigger on an external trigger.  There can only be 4
  // external triggers.
  // Call Add() and SetExternalTrigger() before Start().
  void SetExternalTrigger(DigitalSource *input, bool rising, bool falling);

  // Starts reading samples into the buffer.  Clears all previous samples before
  // starting.
  // Call Start() before Read().
  void Start(size_t queue_depth);

  enum ReadStatus {
    STATUS_OK = 0,
    STATUS_TIMEOUT = 1,
    STATUS_ERROR = 2,
  };

  // Reads a sample from the DMA buffer, waiting up to timeout_ms for it.
  // Returns a status code indicating whether the read worked, timed out, or
  // failed.
  // Call Add() and SetExternalTrigger() then Start() before Read().
  // The sample is only usable while this DMA object is left started.
  ReadStatus Read(DMASample *sample, uint32_t timeout_ms, size_t *remaining);

 private:
  ::std::unique_ptr<nFPGA::tDMAManager> manager_;  // set by Start()
  typedef nFPGA::nRoboRIO_FPGANamespace::tDMA tDMA;
  friend DMASample;

  // The offsets into the sample structure for each DMA type, or -1 if it isn't
  // in the set of values.
  ssize_t channel_offsets_[18];

  // The size of the data to read to get a sample.
  size_t capture_size_ = 0;
  tDMA::tConfig tconfig_;
  tDMA *tdma_config_;

  ::std::array<bool, 4> trigger_channels_ = {{false, false, false, false}};
};

#endif  // _DMA_H_
