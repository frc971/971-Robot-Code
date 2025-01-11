#ifndef FRC971_WPILIB_SWERVE_SWERVE_UTIL_H_
#define FRC971_WPILIB_SWERVE_SWERVE_UTIL_H_

#include "frc971/control_loops/swerve/swerve_drivetrain_position_static.h"
#include "frc971/wpilib/encoder_and_potentiometer.h"
#include "frc971/wpilib/swerve/swerve_constants_static.h"

namespace frc971::wpilib::swerve {

struct SwerveEncoders {
  void set_front_left(std::unique_ptr<frc::Encoder> encoder,
                      std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    front_left.set_encoder(std::move(encoder));
    front_left.set_absolute_pwm(std::move(absolute_pwm));
  }

  void set_front_right(std::unique_ptr<frc::Encoder> encoder,
                       std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    front_right.set_encoder(std::move(encoder));
    front_right.set_absolute_pwm(std::move(absolute_pwm));
  }

  void set_back_left(std::unique_ptr<frc::Encoder> encoder,
                     std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    back_left.set_encoder(std::move(encoder));
    back_left.set_absolute_pwm(std::move(absolute_pwm));
  }

  void set_back_right(std::unique_ptr<frc::Encoder> encoder,
                      std::unique_ptr<frc::DigitalInput> absolute_pwm) {
    back_right.set_encoder(std::move(encoder));
    back_right.set_absolute_pwm(std::move(absolute_pwm));
  }

  // Populates the Position message with the mag encoder values.
  void PopulatePosition(frc971::control_loops::swerve::PositionStatic *fbs,
                        const SwervePositionConstants *constants) {
    std::vector<std::pair<frc971::wpilib::AbsoluteEncoder *,
                          frc971::AbsolutePositionStatic *>>
        encoders;

    encoders.emplace_back(&front_left,
                          fbs->add_front_left()->add_rotation_position());
    encoders.emplace_back(&front_right,
                          fbs->add_front_right()->add_rotation_position());
    encoders.emplace_back(&back_left,
                          fbs->add_back_left()->add_rotation_position());
    encoders.emplace_back(&back_right,
                          fbs->add_back_right()->add_rotation_position());

    for (auto [encoder, fbs_encoder] : encoders) {
      fbs_encoder->set_encoder(encoder->ReadRelativeEncoder() *
                               constants->relative_encoder_scale());
      fbs_encoder->set_absolute_encoder(encoder->ReadAbsoluteEncoder() *
                                        constants->absolute_encoder_scale());
    }
  }

  frc971::wpilib::AbsoluteEncoder front_left;
  frc971::wpilib::AbsoluteEncoder front_right;
  frc971::wpilib::AbsoluteEncoder back_left;
  frc971::wpilib::AbsoluteEncoder back_right;
};

}  // namespace frc971::wpilib::swerve

#endif  // FRC971_WPILIB_SWERVE_SWERVE_UTIL_H_
