#ifndef FRC971_CONTROL_LOOPS_ENCODER_FAULT_DETECTOR_H_
#define FRC971_CONTROL_LOOPS_ENCODER_FAULT_DETECTOR_H_

#include "aos/containers/sized_array.h"
#include "aos/time/time.h"
#include "frc971/control_loops/can_talonfx_generated.h"
#include "frc971/control_loops/encoder_fault_status_generated.h"

namespace frc971 {
namespace control_loops {
// The EncoderFaultDetector class will check for faults within a subsystem by
// taking in an array of CAN encoder positions and an encoder position. When any
// encoder gives a value that does not align with the others, the class will set
// faulted to true
template <uint8_t NumberofMotors>
class EncoderFaultDetector {
 public:
  enum EncoderState {
    SAME,
    INCREASING,
    DECREASING,
  };

  void Iterate(double encoder_position,
               const flatbuffers::Vector<
                   flatbuffers::Offset<frc971::control_loops::CANTalonFX>>
                   *falcon_positions,
               aos::monotonic_clock::time_point current_time);

  void Iterate(double encoder_position,
               aos::SizedArray<double, NumberofMotors> motor_positions,
               aos::monotonic_clock::time_point current_time);

  flatbuffers::Offset<EncoderFaultStatus> PopulateStatus(
      flatbuffers::FlatBufferBuilder *fbb);

  bool isfaulted() { return faulted_; }

 private:
  // The amount of time in milliseconds it will take before new data is ignored
  static constexpr std::chrono::milliseconds kMaxTimeWithNoUpdate{10};
  // Whether the encoders are faulted or not
  bool faulted_ = false;
  // The previous encoder position
  double prev_encoder_position_;
  // The previous motor positions
  aos::SizedArray<double, NumberofMotors> prev_motor_positions_;
  // A timestamp representing the last time data was updated
  aos::monotonic_clock::time_point last_accepted_time_{
      aos::monotonic_clock::min_time};
};

template <uint8_t NumberofMotors>
void EncoderFaultDetector<NumberofMotors>::Iterate(
    double encoder_position,
    const flatbuffers::Vector<
        flatbuffers::Offset<frc971::control_loops::CANTalonFX>>
        *falcon_positions,
    aos::monotonic_clock::time_point current_time) {
  aos::SizedArray<double, NumberofMotors> motors;

  for (const auto &motor : *falcon_positions) {
    motors.push_back(motor->position());
  }

  Iterate(encoder_position, motors, current_time);
}

template <uint8_t NumberofMotors>
void EncoderFaultDetector<NumberofMotors>::Iterate(
    double encoder_position,
    aos::SizedArray<double, NumberofMotors> motor_positions,
    aos::monotonic_clock::time_point current_time) {
  if (prev_motor_positions_.empty()) {
    prev_encoder_position_ = encoder_position;
    prev_motor_positions_ = motor_positions;
    last_accepted_time_ = current_time;
    return;
  }

  aos::monotonic_clock::duration time_elapsed =
      current_time - last_accepted_time_;

  // If more than 10 milliseconds has passed, then do not check for a fault
  if (time_elapsed <= EncoderFaultDetector::kMaxTimeWithNoUpdate) {
    constexpr auto get_encoder_state =
        [](const double now, const double previous) -> EncoderState {
      if (now > previous) {
        return INCREASING;
      }
      if (now < previous) {
        return DECREASING;
      }
      return SAME;
    };

    for (uint64_t i = 0; i < motor_positions.size(); i++) {
      const EncoderState encoder =
          get_encoder_state(encoder_position, prev_encoder_position_);
      const EncoderState motor =
          get_encoder_state(motor_positions[i], prev_motor_positions_[i]);

      if (encoder != motor) {  // If the encoder and the motor states are not
                               // the same set faulted to true
        faulted_ = true;
      }
    }
  }

  prev_encoder_position_ = encoder_position;
  prev_motor_positions_ = motor_positions;
  last_accepted_time_ = current_time;
}

template <uint8_t NumberofMotors>
flatbuffers::Offset<EncoderFaultStatus>
EncoderFaultDetector<NumberofMotors>::PopulateStatus(
    flatbuffers::FlatBufferBuilder *fbb) {
  EncoderFaultStatus::Builder builder(*fbb);
  builder.add_faulted(faulted_);
  return builder.Finish();
}

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_ENCODER_FAULT_DETECTOR_H_