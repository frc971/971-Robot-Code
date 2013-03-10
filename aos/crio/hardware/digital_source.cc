#include "aos/crio/hardware/digital_source.h"

using ::std::unique_ptr;

namespace aos {
namespace crio {
namespace hardware {

AnalogTriggerOutput::AnalogTriggerOutput(unique_ptr< ::AnalogTrigger> trigger,
                                         ::AnalogTriggerOutput::Type type,
                                         float lowerVoltage,
                                         float upperVoltage)
    : trigger_holder_(::std::move(trigger)),
      output_(trigger_holder_->CreateOutput(type)) {
  trigger_holder_->SetLimitsVoltage(lowerVoltage, upperVoltage);
}

}  // namespace hardware
}  // namespace crio
}  // namespace aos
