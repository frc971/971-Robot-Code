#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_ENCODER_FAULT_DETECTOR_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_ENCODER_FAULT_DETECTOR_H_

#include "aos/events/event_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_can_position_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/encoder_fault_detector.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

// This class will be used to detect any encoder faults within the drivetrain
// using values from the left/right encoders and motors

template <uint8_t NumberofDrivetrainMotors>
class DrivetrainEncoderFaultDetector {
 public:
  DrivetrainEncoderFaultDetector(aos::EventLoop *event_loop);

  void UpdateDetectors();

  void SendMessage();

  flatbuffers::Offset<drivetrain::Faults> PopulateFaults(
      flatbuffers::FlatBufferBuilder *fbb);

 private:
  aos::EventLoop *event_loop_;
  EncoderFaultDetector<NumberofDrivetrainMotors> left_encoder_fault_detector_;
  EncoderFaultDetector<NumberofDrivetrainMotors> right_encoder_fault_detector_;
  aos::Sender<frc971::control_loops::drivetrain::Status>
      drivetrain_status_sender_;
  aos::Fetcher<control_loops::drivetrain::Position>
      drivetrain_position_fetcher_;
  aos::Fetcher<control_loops::drivetrain::CANPosition>
      drivetrain_can_position_fetcher_;
};

template <uint8_t NumberofDrivetrainMotors>
DrivetrainEncoderFaultDetector<NumberofDrivetrainMotors>::
    DrivetrainEncoderFaultDetector(aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      drivetrain_status_sender_(
          event_loop->MakeSender<drivetrain::Status>("/drivetrain")),
      drivetrain_position_fetcher_(
          event_loop->MakeFetcher<drivetrain::Position>("/drivetrain")),
      drivetrain_can_position_fetcher_(
          event_loop->MakeFetcher<drivetrain::CANPosition>("/drivetrain")) {
  event_loop
      ->AddTimer([this]() {
        UpdateDetectors();
        SendMessage();
      })
      ->Schedule(event_loop->monotonic_now(),
                 std::chrono::duration_cast<aos::monotonic_clock::duration>(
                     std::chrono::milliseconds(5)));
}

template <uint8_t NumberofDrivetrainMotors>
void DrivetrainEncoderFaultDetector<
    NumberofDrivetrainMotors>::UpdateDetectors() {
  drivetrain_position_fetcher_.Fetch();
  drivetrain_can_position_fetcher_.Fetch();

  if (drivetrain_position_fetcher_.get() &&
      drivetrain_can_position_fetcher_.get()) {
    left_encoder_fault_detector_.Iterate(
        drivetrain_position_fetcher_->left_encoder(),
        drivetrain_can_position_fetcher_->left_falcons(),
        event_loop_->monotonic_now());
    right_encoder_fault_detector_.Iterate(
        drivetrain_position_fetcher_->right_encoder(),
        drivetrain_can_position_fetcher_->right_falcons(),
        event_loop_->monotonic_now());
  }
}

template <uint8_t NumberofDrivetrainMotors>
void DrivetrainEncoderFaultDetector<NumberofDrivetrainMotors>::SendMessage() {
  auto builder = drivetrain_status_sender_.MakeBuilder();
  auto offset = PopulateFaults(builder.fbb());

  drivetrain::Status::Builder drivetrain_status_builder =
      builder.MakeBuilder<drivetrain::Status>();

  drivetrain_status_builder.add_encoder_faults(offset);
  builder.CheckOk(builder.Send(drivetrain_status_builder.Finish()));
}

template <uint8_t NumberofDrivetrainMotors>
flatbuffers::Offset<drivetrain::Faults>
DrivetrainEncoderFaultDetector<NumberofDrivetrainMotors>::PopulateFaults(
    flatbuffers::FlatBufferBuilder *fbb) {
  flatbuffers::Offset<EncoderFaultStatus> left_encoder_fault_detector_offset =
      left_encoder_fault_detector_.PopulateStatus(fbb);
  flatbuffers::Offset<EncoderFaultStatus> right_encoder_fault_detector_offset =
      right_encoder_fault_detector_.PopulateStatus(fbb);

  drivetrain::Faults::Builder drivetrain_faults_builder(*fbb);

  drivetrain_faults_builder.add_left_faulted(
      left_encoder_fault_detector_offset);
  drivetrain_faults_builder.add_right_faulted(
      right_encoder_fault_detector_offset);

  return drivetrain_faults_builder.Finish();
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif