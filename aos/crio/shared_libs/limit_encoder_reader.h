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
#include "aos/crio/hardware/digital_source.h"
#include "aos/crio/hardware/counter.h"
// TODO(brians): fix the build file

namespace aos {
namespace crio {

// This class handles reading an Encoder's value each time a digital sensor
// triggers, including all of the irritating WPILib setup and synchronization.
class LimitEncoderReader {
 public:
  // See InterruptNotifier for details about the state of the sensor object
  // before the constructor is called.
  LimitEncoderReader(::std::unique_ptr<::aos::crio::hardware::Counter> counter,
                     ::std::uniuqe_ptr<::aos::crio::hardware::DigitalSource>
                         source,
                     bool posEdge, bool negEdge);

  // Retrieves the values. See ZeroSwitchValue's declaration for an explanation
  // of why retrieving all of them at once is necessary.
  ZeroSwitchValue Get();

  // Calls Start() on all owned objects.
  void Start();

  // Only to set things up etc. Getting values through these methods will always
  // have race conditions!
  // Also helpful for debugging.
  const ::std::unique_ptr<::aos::crio::hardware::Counter> &counter() const {
    return counter_;
  }
  const ::std::unique_ptr<::aos::crio::hardware::DigitalSource>
      &source() const {
    return source_;
  }
  
 private:
  static void ReadValueStatic(LimitEncoderReader *self) {
    self->ReadValue();
  }
  void ReadValue();

  ::std::unique_ptr<InterruptNotifier<LimitEncoderReader>> notifier_;
  const ::std::unique_ptr<CountGetter> encoder_;

  ::std::unique_ptr<::aos::crio::hardware::Counter> counter_;
  ::std::unique_ptr<::aos::crio::hardware::DigitalSource> source_;

  // The most recently read value.
  int32_t value_;
  Mutex value_sync_;

  DISALLOW_COPY_AND_ASSIGN(LimitEncoderReader);
};

}  // namespace crio
}  // namespace aos

#endif  // AOS_CRIO_SHARED_LIBS_ENCODER_READER_H_
