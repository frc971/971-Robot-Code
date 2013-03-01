#ifndef AOS_CRIO_SHARED_LIBS_ENCODER_READER_H_
#define AOS_CRIO_SHARED_LIBS_ENCODER_READER_H_

#include "aos/common/libstdc++/memory"

#include "WPILib/DigitalSource.h"
#include "WPILib/AnalogChannel.h"
#include "WPILib/AnalogTriggerOutput.h"
#include "WPILib/DigitalInput.h"
#include "WPILib/Encoder.h"

#include "aos/common/mutex.h"
#include "aos/common/zero_switch_value.h"
#include "aos/common/macros.h"
#include "aos/crio/shared_libs/interrupt_notifier.h"

namespace aos {
namespace crio {

// This class handles reading an Encoder's value each time a digital sensor
// triggers, including all of the irritating WPILib setup and synchronization.
class LimitEncoderReader {
 public:
  // Defaults for the voltages for AnalogTriggers. They work well for digital
  // sensors connected to analog inputs.
  // TODO(brians): make sure these values are reasonable
  static const float kDefaultLowerVoltage = 1;
  static const float kDefaultUpperVoltage = 3;

  // See InterruptNotifier for details about the state of the sensor object
  // before the constructor is called.
  // The different constructors take in different types of inputs and configure
  // them to give interrupts.
  //
  // type is the type for triggering interrupts while triggeredType is the one
  // to use for writing *triggered in Get().
  // AnalogTriggerOutput::Type::k*Pulse don't seem to do anything...
  LimitEncoderReader(const ::std::unique_ptr<Encoder> &encoder,
                     ::std::unique_ptr<AnalogChannel> channel,
                     AnalogTriggerOutput::Type type,
                     AnalogTriggerOutput::Type triggeredType,
                     bool posEdge, bool negEdge,
                     float lowerVoltage = kDefaultLowerVoltage,
                     float upperVoltage = kDefaultUpperVoltage);
  LimitEncoderReader(const ::std::unique_ptr<Encoder> &encoder,
                     ::std::unique_ptr<DigitalInput> sensor,
                     bool posEdge, bool negEdge);

  // Retrieves the values. See ZeroSwitchValue's declaration for an explanation
  // of why retrieving all of them is necessary.
  ZeroSwitchValue Get();

  // Calls Start() on all contained objects.
  void Start();

  // Only to set things up etc. Getting values through these methods will always
  // have race conditions!
  // Also helpful for debugging.
  const ::std::unique_ptr<Encoder> &encoder() const { return encoder_; }
  const ::std::unique_ptr<DigitalSource> &source() const { return source_; }
  
 private:
  // A class to deal with the fact that WPILib's AnalogTriggerOutput and
  // DigitalInput have no common superclass with their Get() functions.
  // Also handles taking ownership of some attached objects for
  // AnalogTriggerOutput.
  class OnOffGetter {
   public:
    virtual bool Get() = 0;

    virtual ~OnOffGetter() {}
  };
  class AnalogOnOffGetter;
  class DigitalOnOffGetter;

  // The common initialization code.
  // Gets called by all of the constructors.
  // getter_, encoder_, and source_ must be set before calling this.
  void Init(bool posEdge, bool negEdge);

  static void ReadValueStatic(LimitEncoderReader *self) {
    self->ReadValue();
  }
  void ReadValue();

  ::std::unique_ptr<InterruptNotifier<LimitEncoderReader>> notifier_;
  const ::std::unique_ptr<Encoder> &encoder_;

  ::std::unique_ptr<OnOffGetter> getter_;
  ::std::unique_ptr<DigitalSource> source_;

  // The most recently read value.
  int32_t value_;
  Mutex value_sync_;

  DISALLOW_COPY_AND_ASSIGN(LimitEncoderReader);
};

}  // namespace crio
}  // namespace aos

#endif  // AOS_CRIO_SHARED_LIBS_ENCODER_READER_H_
