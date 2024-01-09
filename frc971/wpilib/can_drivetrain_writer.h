#include "frc971/can_configuration_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/talonfx.h"

namespace frc971 {
namespace wpilib {

class CANDrivetrainWriter : public ::frc971::wpilib::LoopOutputHandler<
                                ::frc971::control_loops::drivetrain::Output> {
 public:
  CANDrivetrainWriter(::aos::EventLoop *event_loop);

  void set_talonfxs(std::vector<std::shared_ptr<TalonFX>> right_talonfxs,
                    std::vector<std::shared_ptr<TalonFX>> left_talonfxs);

  void HandleCANConfiguration(const CANConfiguration &configuration);

  static constexpr int kDrivetrainWriterPriority = 35;

 private:
  void WriteConfigs();

  void Write(
      const ::frc971::control_loops::drivetrain::Output &output) override;

  void Stop() override;

  std::vector<std::shared_ptr<TalonFX>> right_talonfxs_;
  std::vector<std::shared_ptr<TalonFX>> left_talonfxs_;
};

}  // namespace wpilib
}  // namespace frc971
