#ifndef AOS_CRIO_HARDWARE_DIGITAL_SOURCE_H_
#define AOS_CRIO_HARDWARE_DIGITAL_SOURCE_H_

#include "aos/common/libstdc++/memory"

#include "WPILib/DigitalSource.h"
#include "WPILib/DigitalInput.h"
#include "WPILib/AnalogTrigger.h"
#include "WPILib/AnalogTriggerOutput.h"

#include "aos/common/macros.h"

namespace aos {
namespace crio {
namespace hardware {

// Wrapper for WPILib's class of the same name. Provides an actual Get()
// function and makes creating analog ones easier.
class DigitalSource {
 public:
  virtual ~DigitalSource() {}

  virtual bool Get() = 0;
  // This object maintains ownership.
  virtual ::DigitalSource *source() const = 0;

 protected:
  DigitalSource() {}

 private:
  DISALLOW_COPY_AND_ASSIGN(DigitalSource);
};

class AnalogTriggerOutput : public DigitalSource {
 public:
  // Defaults for the voltages for AnalogTriggers. They work well for digital
  // sensors connected to analog inputs.
  static const float kDefaultLowerVoltage = 1.35;
  static const float kDefaultUpperVoltage = 4;

  // Will set up the voltages on trigger.
  // Takes ownership of trigger to make sure it stays around so that the output
  // it creates won't blow up (because it holds on to and uses it).
  AnalogTriggerOutput(::std::unique_ptr< ::AnalogTrigger> trigger,
                      ::AnalogTriggerOutput::Type type,
                      float lowerVoltage = kDefaultLowerVoltage,
                      float upperVoltage = kDefaultUpperVoltage);
  explicit AnalogTriggerOutput(::std::unique_ptr< ::AnalogTriggerOutput> output)
      : output_(::std::move(output)) {}

  virtual bool Get() { return output_->Get(); }
  virtual ::DigitalSource *source() const { return output_.get(); }

 private:
  const ::std::unique_ptr< ::AnalogTrigger> trigger_holder_;

  const ::std::unique_ptr< ::AnalogTriggerOutput> output_;
};

class DigitalInput : public DigitalSource {
 public:
  explicit DigitalInput(uint32_t channel)
      : input_(::std::unique_ptr< ::DigitalInput>(
              new ::DigitalInput(channel))) {
  }
  DigitalInput(uint8_t module, uint32_t channel)
      : input_(::std::unique_ptr< ::DigitalInput>(
              new ::DigitalInput(module, channel))) {
  }
  explicit DigitalInput(::std::unique_ptr< ::DigitalInput> input)
      : input_(::std::move(input)) {}

  virtual bool Get() { return input_->Get(); }
  virtual ::DigitalSource *source() const { return input_.get(); }

 private:
  const ::std::unique_ptr< ::DigitalInput> input_;
};

}  // namespace hardware
}  // namespace crio
}  // namespace aos

#endif  // AOS_CRIO_HARDWARE_DIGITAL_SOURCE_H_
