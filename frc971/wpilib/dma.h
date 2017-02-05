#ifndef FRC971_WPILIB_DMA_H_
#define FRC971_WPILIB_DMA_H_

// Interface to the roboRIO FPGA's DMA features.
// TODO(Brian): Make this less wpilib-like and more frc971-like.

#include <stdint.h>

#include <array>
#include <memory>

#ifdef WPILIB2017
#include "HAL/ChipObject.h"
#else
#include "ChipObject.h"
#endif
#include "ErrorBase.h"

class DMA;
#ifdef WPILIB2017
namespace frc {
class DigitalSource;
class AnalogInput;
class Encoder;
}  // namespace frc
#else
class DigitalSource;
class AnalogInput;
class Encoder;
namespace frc {
using ::DigitalSource;
using ::AnalogInput;
using ::Encoder;
}  // namespace frc
#endif

// A POD class which stores the data from a DMA sample and provides safe ways to
// access it.
class DMASample {
 public:
  DMASample() = default;

  // Returns the FPGA timestamp of the sample in seconds.
  double GetTimestamp() const;
  // Returns the FPGA timestamp of the sample in microseconds.
  uint32_t GetTime() const;

  // All Get methods either return the requested value, or set the Error.

  // Returns the value of the digital input in the sample.
  bool Get(DigitalSource *input) const;
  // Returns the raw value of the encoder in the sample.
  int32_t GetRaw(Encoder *input) const;
  // Returns the {1, 2, or 4} X scaled value of the encoder in the sample.
  int32_t Get(Encoder *input) const;
  // Returns the raw 12-bit value from the ADC.
  uint16_t GetValue(AnalogInput *input) const;
  // Returns the scaled value of an analog input.
  float GetVoltage(AnalogInput *input) const;

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

// TODO(austin): ErrorBase...
class DMA : public ErrorBase {
 public:
  DMA();
  virtual ~DMA();

  // Sets whether or not DMA is paused.
  // If not specified, the default is false.
  void SetPause(bool pause);

  // Sets the number of triggers that need to occur before a sample is saved.
  // If not specified, the default is 1.
  void SetRate(uint32_t cycles);

  // Adds the input signal to the state to snapshot on the trigger event.
  // It is safe to add the same input multiple times, but there is currently
  // no way to remove one once it has been added.
  // Call Add() and SetExternalTrigger() before Start().
  void Add(Encoder *encoder);
  void Add(DigitalSource *input);
  void Add(AnalogInput *input);

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
  // Returns in *remaining_out the number of DMA samples still queued after this
  // Read().
  // Call Add() and SetExternalTrigger() then Start() before Read().
  // The sample is only usable while this DMA object is left started.
  ReadStatus Read(DMASample *sample, uint32_t timeout_ms,
                  size_t *remaining_out);

  // Translates a ReadStatus code to a string name.
  static const char *NameOfReadStatus(ReadStatus s);

 private:
  ::std::unique_ptr<nFPGA::tDMAManager> manager_;  // set by Start()
  typedef nFPGA::nRoboRIO_FPGANamespace::tDMA tDMA;
  friend DMASample;

  // The offsets into the sample structure for each DMA type, or -1 if it isn't
  // in the set of values.
#ifdef WPILIB2015
  ssize_t channel_offsets_[18];
#else
  ssize_t channel_offsets_[20];
#endif

  // The size of the data to read to get a sample.
  size_t capture_size_ = 0;
  tDMA::tConfig tconfig_;
  tDMA *tdma_config_;

#ifndef WPILIB2015
  ::std::array<bool, 8> trigger_channels_ = {
      {false, false, false, false, false, false, false, false}};
#else
  ::std::array<bool, 4> trigger_channels_ = {{false, false, false, false}};
#endif
};

#endif  // FRC971_WPILIB_DMA_H_
