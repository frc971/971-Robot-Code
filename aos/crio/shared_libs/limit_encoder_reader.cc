#include "aos/crio/shared_libs/limit_encoder_reader.h"

#include <functional>

#include "WPILib/AnalogTrigger.h"

using ::std::unique_ptr;

namespace aos {
namespace crio {

class LimitEncoderReader::AnalogOnOffGetter
    : public LimitEncoderReader::OnOffGetter {
 public:
  AnalogOnOffGetter(unique_ptr<AnalogChannel> channel,
                    unique_ptr<AnalogTrigger> trigger,
                    unique_ptr<AnalogTriggerOutput> source)
      : channel_(::std::move(channel)), trigger_(::std::move(trigger)),
        source_(::std::move(source)) {}

  virtual bool Get() {
    return source_->Get();
  }

 private:
  unique_ptr<AnalogChannel> channel_;
  unique_ptr<AnalogTrigger> trigger_;
  unique_ptr<AnalogTriggerOutput> source_;
};

class LimitEncoderReader::DigitalOnOffGetter
    : public LimitEncoderReader::OnOffGetter {
 public:
  DigitalOnOffGetter(const unique_ptr<DigitalInput> &source)
      : source_(source) {}

  virtual bool Get() {
    return source_->Get();
  }

 private:
  const unique_ptr<DigitalInput> &source_;
};

int32_t LimitEncoderReader::EncoderCountGetter::Get() {
  return counter_->GetRaw();
}

int32_t LimitEncoderReader::CounterCountGetter::Get() {
  return counter_->Get();
}

void LimitEncoderReader::Init(unique_ptr<AnalogChannel> channel,
                         AnalogTriggerOutput::Type type,
                         AnalogTriggerOutput::Type triggeredType,
                         bool posEdge, bool negEdge,
                         float lowerVoltage,
                         float upperVoltage) {
  unique_ptr<AnalogTrigger> trigger(new AnalogTrigger(channel.get()));
  trigger->SetLimitsVoltage(lowerVoltage, upperVoltage);
  source_ = unique_ptr<AnalogTriggerOutput>(trigger->CreateOutput(type));
  unique_ptr<AnalogTriggerOutput> getter_output(
      trigger->CreateOutput(triggeredType));
  getter_ = unique_ptr<AnalogOnOffGetter>(
      new AnalogOnOffGetter(::std::move(channel), ::std::move(trigger),
                            ::std::move(getter_output)));
  CommonInit(posEdge, negEdge);
}

void LimitEncoderReader::Init(unique_ptr<DigitalInput> source,
                         bool posEdge, bool negEdge) {
  getter_ = unique_ptr<DigitalOnOffGetter>(new DigitalOnOffGetter(source));
  source_ = ::std::move(source);
  CommonInit(posEdge, negEdge);
}

void LimitEncoderReader::CommonInit(bool posEdge, bool negEdge) {
  notifier_ = unique_ptr<InterruptNotifier<LimitEncoderReader>>(
      new InterruptNotifier<LimitEncoderReader>(ReadValueStatic,
                                                source_.get(),
                                                this));
  source_->SetUpSourceEdge(posEdge, negEdge);
}

void LimitEncoderReader::ReadValue() {
  // Retrieve the value before potentially waiting for somebody else currently
  // looking at the saved one.
  int32_t new_value = encoder_->Get();
  {
    MutexLocker locker(&value_sync_);
    value_ = new_value;
  }
}

ZeroSwitchValue LimitEncoderReader::Get() {
  MutexLocker locker(&value_sync_);
  return ZeroSwitchValue{encoder_->Get(), value_, getter_->Get()};
}

void LimitEncoderReader::Start() {
  notifier_->Start();
}

}  // namespace crio
}  // namespace aos
