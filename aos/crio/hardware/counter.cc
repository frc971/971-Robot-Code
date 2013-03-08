#include "aos/crio/hardware/counter.h"

#include "aos/crio/hardware/digital_source.h"
#include "aos/common/logging/logging.h"

using ::std::unique_ptr;

namespace aos {
namespace crio {
namespace hardware {

int Counter::GetDenominator() {
  switch (type_) {
    case ::CounterBase::EncodingType::k1X:
      return 1;
    case ::CounterBase::EncodingType::k2X:
      return 2;
    case ::CounterBase::EncodingType::k4X:
      return 4;
  }
  BadEncodingType(type_);
}

void Counter::BadEncodingType(::CounterBase::EncodingType type) {
  if (type == ::CounterBase::EncodingType::k1X ||
      type == ::CounterBase::EncodingType::k2X ||
      type == ::CounterBase::EncodingType::k4X) {
    LOG(FATAL, "somebody didn't handle all of the possible encoding types!\n");
  }
  LOG(FATAL, "bad ::CounterBase::EncodingType %d\n", static_cast<int>(type));
}

unique_ptr<Counter> Counter::Create(unique_ptr< ::DigitalSource> a,
                                    unique_ptr< ::DigitalSource> b,
                                    ::CounterBase::EncodingType type) {
  switch (type) {
    case ::CounterBase::EncodingType::k4X:
      return unique_ptr<Counter>(new EncoderCounter(::std::move(a),
                                                    ::std::move(b)));
    case ::CounterBase::EncodingType::k2X:
    case ::CounterBase::EncodingType::k1X:
      return unique_ptr<Counter>(new CounterCounter(::std::move(a),
                                                    ::std::move(b),
                                                    type));
  }
  BadEncodingType(type);
}

EncoderCounter::EncoderCounter(unique_ptr< ::DigitalSource> a_source,
                               unique_ptr< ::DigitalSource> b_source)
    : Counter(::std::move(a_source), ::std::move(b_source),
              ::CounterBase::EncodingType::k4X),
      encoder_(new ::Encoder(a().get(), b().get())) {}

CounterCounter::CounterCounter(unique_ptr< ::DigitalSource> a_source,
                               unique_ptr< ::DigitalSource> b_source,
                               ::CounterBase::EncodingType type)
    : Counter(::std::move(a_source), ::std::move(b_source), type),
      counter_(new ::Counter(type, a().get(), b().get(), false /*inverted*/)) {
  assert(type == ::CounterBase::EncodingType::k1X ||
         type == ::CounterBase::EncodingType::k2X);
}

}  // namespace hardware
}  // namespace crio
}  // namespace aos
