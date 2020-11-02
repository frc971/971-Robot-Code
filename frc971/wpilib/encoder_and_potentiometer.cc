#include "frc971/wpilib/encoder_and_potentiometer.h"

#include "aos/logging/logging.h"
#include "aos/realtime.h"

namespace frc971 {
namespace wpilib {

bool DMAEncoder::DoUpdateFromSample(const DMASample &sample) {
  if (index_last_value_) {
    // It was already true last time, so check if it's reset back to false yet.
    index_last_value_ = sample.Get(index_.get());
  } else if (sample.Get(index_.get())) {
    // This sample is posedge, so record all the values.
    index_last_value_ = true;
    ++index_posedge_count_;
    last_encoder_value_ = sample.GetRaw(encoder_.get());
    return true;
  }
  return false;
}

}  // namespace wpilib
}  // namespace frc971
