#include "aos/crio/hardware/hardware.h"

#include "aos/crio/hardware/digital_source.h"

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
}

unique_ptr<Counter> Counter::Create(unique_ptr<DigitalSource> a,
                                    unique_ptr<DigitalSource> b,
                                    ::CounterBase::EncodingType type) {
  switch (type) {
    case ::CounterBase::EncodingType::k4X:
      return unique_ptr<Counter>(new EncoderCounter(::std::move(a),
                                                    ::std::move(b)));
    case ::CounterBase::EncodingTYpe::k2X:
    case ::CounterBase::EncodingType::k1X:
      return unique_ptr<Counter>(new CounterCounter(::std::move(a),
                                                    ::std::move(b),
                                                    type));
  }
}

EncoderCounter::EncoderCounter(unique_ptr<DigitalSource> a,
                               unique_ptr<DigitalSource> b)
    : Counter(::std::move(a), ::std::move(b), ::CounterBase::EncodingType::k4X),
      encoder_(new ::Encoder(a()->source(), b()->source())) {}

CounterCounter::CounterCounter(unique_ptr<DigitalSource> a,
                               unique_ptr<DigitalSource> b,
                               ::CounterBase::EncodingType type)
    : Counter(::std::move(a), ::std::move(b), type),
      counter_(new ::Counter(type, a()->source(), b()->source(),
                             false /*inverted*/)) {
  assert(type == ::CounterBase::EncodingType::k1X ||
         type == ::CounterBase::EncodingType::k2X);
}

}  // namespace hardware
}  // namespace crio
}  // namespace aos
