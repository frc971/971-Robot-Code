#include "aos/crio/shared_libs/limit_encoder_reader.h"

#include <functional>

using ::std::unique_ptr;
namespace hardware = ::aos::crio::hardware;

namespace aos {
namespace crio {

LimitEncoderReader::LimitEncoderReader(unique_ptr<::hardware::Counter> &counter,
                                       unique_ptr<::hardware::DigitalSource>
                                           source,
                                       bool posEdge, bool negEdge)
    : counter_(counter), source(::std::move(source)),
      notifier_(ReadValueStatic, source_->source(), this) {
  source_->source()->SetUpSourceEdge(posEdge, negEdge);
}

void LimitEncoderReader::ReadValue() {
  // Retrieve the value before potentially waiting for somebody else currently
  // looking at the saved one.
  int32_t new_value = counter_->Get();
  {
    MutexLocker locker(&value_sync_);
    value_ = new_value;
  }
}

ZeroSwitchValue LimitEncoderReader::Get() {
  MutexLocker locker(&value_sync_);
  return ZeroSwitchValue{counter_->Get(), value_, source_->Get()};
}

void LimitEncoderReader::Start() {
  notifier_->Start();
}

}  // namespace crio
}  // namespace aos
