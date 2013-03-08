#ifndef AOS_CRIO_HARDWARE_ENCODER_H_
#define AOS_CRIO_HARDWARE_ENCODER_H_

#include "aos/common/libstdc++/memory"

#include "WPILib/Counter.h"
#include "WPILib/CounterBase.h"
#include "WPILib/Encoder.h"

#include "aos/common/macros.h"

namespace aos {
namespace crio {
namespace hardware {

class DigitalSource;

// Wrapper for WPILib's CounterBase that implements Get() consistently.
// WPILib doesn't really differentiate between (and actually confuses) ticks and
// cycles. There is 1 tick for each input edge that a Counter sees and 1 cycle
// every time the inputs repeat their pattern.
// The constructors for both of them are such a mess that these classes
// intentionally do not support using them. Instead, just call the constructors
// for these classes and let them handle using the correct constructor.
class Counter {
 public:
  virtual ~Counter() {}

  // Returns the number of ticks (NOT divided by the number of ticks per
  // cycle like WPILib _sometimes_ is!).
  virtual int32_t Get() = 0;
  // This object maintains ownership.
  virtual ::CounterBase *counter_base() const = 0;

  const ::std::unique_ptr< ::DigitalSource> &a() { return a_; }
  const ::std::unique_ptr< ::DigitalSource> &b() { return b_; }

  // Returns the denominator to convert from ticks to cycles.
  int GetDenominator();

  // Will create an instance of a subclass as appropriate for type.
  // This should be used (except for special circumstances) for constructing all
  // instances because it makes it much easier to change the encoding type.
  static ::std::unique_ptr<Counter> Create(::std::unique_ptr< ::DigitalSource>
                                               a,
                                           ::std::unique_ptr< ::DigitalSource>
                                               b,
                                           ::CounterBase::EncodingType type);

 protected:
  Counter(::std::unique_ptr< ::DigitalSource> a,
          ::std::unique_ptr< ::DigitalSource> b,
          ::CounterBase::EncodingType type)
      : a_(::std::move(a)),
        b_(::std::move(b)),
        type_(type) {}

  // What to do at the end of functions that handle all encoding types to make
  // GCC happy. Will LOG(FATAL) a message.
  static void BadEncodingType(::CounterBase::EncodingType type)
      __attribute__((noreturn));

 private:
  const ::std::unique_ptr< ::DigitalSource> a_;
  const ::std::unique_ptr< ::DigitalSource> b_;

  // Because WPILib doesn't actually keep track of it...
  const ::CounterBase::EncodingType type_;

  DISALLOW_COPY_AND_ASSIGN(Counter);
};

// Only allows creating one with k4X decoding because otherwise ::Encoder just
// creates an internal ::Counter, which is really stupid.
class EncoderCounter : public Counter {
 public:
  EncoderCounter(::std::unique_ptr< ::DigitalSource> a,
                 ::std::unique_ptr< ::DigitalSource> b);

  virtual int32_t Get() { return encoder_->GetRaw(); }
  virtual ::CounterBase *counter_base() const { return encoder_.get(); }
  const ::std::unique_ptr< ::Encoder> &encoder() { return encoder_; }

 private:
  const ::std::unique_ptr< ::Encoder> encoder_;
};

class CounterCounter : public Counter {
 public:
  CounterCounter(::std::unique_ptr< ::DigitalSource> a,
                 ::std::unique_ptr< ::DigitalSource> b,
                 ::CounterBase::EncodingType type);

  virtual int32_t Get() { return counter_->Get(); }
  virtual ::CounterBase *counter_base() const { return counter_.get(); }
  const ::std::unique_ptr< ::Counter> &counter() { return counter_; }

 private:
  const ::std::unique_ptr< ::Counter> counter_;
};

}  // namespace hardware
}  // namespace crio
}  // namespace aos

#endif  // AOS_CRIO_HARDWARE_ENCODER_H_
