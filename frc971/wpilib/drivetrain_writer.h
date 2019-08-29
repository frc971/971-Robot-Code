#ifndef FRC971_WPILIB_DRIVETRAIN_WRITER_H_
#define FRC971_WPILIB_DRIVETRAIN_WRITER_H_

#include "aos/commonmath.h"

#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/wpilib/ahal/PWM.h"
#include "frc971/wpilib/loop_output_handler.h"

namespace frc971 {
namespace wpilib {

class DrivetrainWriter : public ::frc971::wpilib::LoopOutputHandler<
                             ::frc971::control_loops::drivetrain::Output> {
 public:
  DrivetrainWriter(::aos::EventLoop *event_loop)
      : ::frc971::wpilib::LoopOutputHandler<
            ::frc971::control_loops::drivetrain::Output>(event_loop,
                                                         "/drivetrain") {}

  void set_left_controller0(::std::unique_ptr<::frc::PWM> t, bool reversed) {
    left_controller0_ = ::std::move(t);
    reversed_left0_ = reversed;
  }

  void set_right_controller0(::std::unique_ptr<::frc::PWM> t, bool reversed) {
    right_controller0_ = ::std::move(t);
    reversed_right0_ = reversed;
  }

  void set_left_controller1(::std::unique_ptr<::frc::PWM> t, bool reversed) {
    left_controller1_ = ::std::move(t);
    reversed_left1_ = reversed;
  }

  void set_right_controller1(::std::unique_ptr<::frc::PWM> t, bool reversed) {
    right_controller1_ = ::std::move(t);
    reversed_right1_ = reversed;
  }

 private:
  void Write(
      const ::frc971::control_loops::drivetrain::Output &output) override;
  void Stop() override;

  double SafeSpeed(bool reversed, double voltage) {
    return (::aos::Clip((reversed ? -1.0 : 1.0) * voltage, -12.0, 12.0) / 12.0);
  }

  ::std::unique_ptr<::frc::PWM> left_controller0_, right_controller0_,
      left_controller1_, right_controller1_;

  bool reversed_right0_, reversed_left0_, reversed_right1_, reversed_left1_;
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_DRIVETRAIN_WRITER_H_
