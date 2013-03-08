#include "aos/crio/hardware/digital_source.h"

using ::std::unique_ptr;

namespace aos {
namespace crio {
namespace hardware {

AnalogTriggerOutput::AnalogTriggerOutput(const unique_ptr< ::AnalogTrigger>
                                             &trigger,
                                         ::AnalogTriggerOutput::Type type,
                                         float lowerVoltage,
                                         float upperVoltage)
    : output_(trigger->CreateOutput(type)) {
  trigger->SetLimitsVoltage(lowerVoltage, upperVoltage);
}

}  // namespace hardware
}  // namespace crio
}  // namespace aos
