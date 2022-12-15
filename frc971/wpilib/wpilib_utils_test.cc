#include "frc971/wpilib/wpilib_utils.h"

#include "aos/testing/flatbuffer_eq.h"
#include "aos/testing/test_logging.h"
#include "frc971/constants.h"

namespace frc971 {
namespace wpilib {
namespace testing {
namespace {

double climber_pot_translate_inverse_test(double position) {
  return position * 10.0;
}

double intake_pot_translate_inverse_test(double position) {
  return position * 2.0;
}

TEST(WpilibUtilsTest, ZeroOffsetZeroBuffer) {
  {
    // Physical pot range: 0.0 to 0.5
    constexpr double kPotentiometerOffset = 0.00;
    constexpr double kLimitBuffer = 0.00;

    ::frc971::constants::Range subsystem_range;
    subsystem_range = ::frc971::constants::Range{
        .lower_hard = 0.01, .upper_hard = 0.49, .lower = 0.1, .upper = 0.4};
    EXPECT_TRUE(SafePotVoltageRange(subsystem_range, kPotentiometerOffset,
                                    climber_pot_translate_inverse_test, false,
                                    kLimitBuffer));
  }
  {
    // Physical pot range: 0.0 to 0.5
    constexpr double kPotentiometerOffset = 0.00;
    constexpr double kLimitBuffer = 0.00;

    ::frc971::constants::Range subsystem_range;
    subsystem_range = ::frc971::constants::Range{
        .lower_hard = 0.0, .upper_hard = 0.5, .lower = 0.1, .upper = 0.4};
    EXPECT_FALSE(SafePotVoltageRange(subsystem_range, kPotentiometerOffset,
                                     climber_pot_translate_inverse_test, false,
                                     kLimitBuffer));
  }
}

TEST(WpilibUtilsTest, LimitBuffer) {
  {
    // Physical pot range: 0.0 to 0.5
    constexpr double kPotentiometerOffset = 0.0;
    constexpr double kLimitBuffer = 0.00;

    ::frc971::constants::Range subsystem_range;
    subsystem_range = ::frc971::constants::Range{
        .lower_hard = 0.005, .upper_hard = 0.495, .lower = 0.1, .upper = 0.4};
    EXPECT_TRUE(SafePotVoltageRange(subsystem_range, kPotentiometerOffset,
                                    climber_pot_translate_inverse_test, false,
                                    kLimitBuffer));
  }
  {
    // Physical pot range: 0.0 to 0.5, with limit buffer: 0.005 to 0.495
    constexpr double kPotentiometerOffset = 0.0;
    constexpr double kLimitBuffer = 0.05;

    ::frc971::constants::Range subsystem_range;
    subsystem_range = ::frc971::constants::Range{
        .lower_hard = 0.005, .upper_hard = 0.495, .lower = 0.1, .upper = 0.4};
    EXPECT_FALSE(SafePotVoltageRange(subsystem_range, kPotentiometerOffset,
                                     climber_pot_translate_inverse_test, false,
                                     kLimitBuffer));
  }
}

TEST(WpilibUtilsTest, PotOffset) {
  {
    // Physical pot range: 0.0 to 0.5
    constexpr double kPotentiometerOffset = 0.0;
    constexpr double kLimitBuffer = 0.00;

    ::frc971::constants::Range subsystem_range;
    subsystem_range = ::frc971::constants::Range{
        .lower_hard = 0.01, .upper_hard = 0.49, .lower = 0.1, .upper = 0.4};
    EXPECT_TRUE(SafePotVoltageRange(subsystem_range, kPotentiometerOffset,
                                    climber_pot_translate_inverse_test, false,
                                    kLimitBuffer));
  }
  {
    // Physical pot range: 0.1 to 0.6
    constexpr double kPotentiometerOffset = 0.1;
    constexpr double kLimitBuffer = 0.00;

    ::frc971::constants::Range subsystem_range;
    subsystem_range = ::frc971::constants::Range{
        .lower_hard = 0.01, .upper_hard = 0.49, .lower = 0.1, .upper = 0.4};
    EXPECT_FALSE(SafePotVoltageRange(subsystem_range, kPotentiometerOffset,
                                     climber_pot_translate_inverse_test, false,
                                     kLimitBuffer));
  }
}

TEST(WpilibUtilsTest, TranslateFunction) {
  {
    // Physical pot range: 0.0 to 2.5
    constexpr double kPotentiometerOffset = 0.0;
    constexpr double kLimitBuffer = 0.00;

    ::frc971::constants::Range subsystem_range;
    subsystem_range = ::frc971::constants::Range{
        .lower_hard = 0.01, .upper_hard = 2.49, .lower = 0.1, .upper = 2.4};
    EXPECT_TRUE(SafePotVoltageRange(subsystem_range, kPotentiometerOffset,
                                    intake_pot_translate_inverse_test, false,
                                    kLimitBuffer));
  }
  {
    // Physical pot range: 0.0 to 0.5
    constexpr double kPotentiometerOffset = 0.0;
    constexpr double kLimitBuffer = 0.00;

    ::frc971::constants::Range subsystem_range;
    subsystem_range = ::frc971::constants::Range{
        .lower_hard = 0.01, .upper_hard = 2.49, .lower = 0.1, .upper = 2.4};
    EXPECT_FALSE(SafePotVoltageRange(subsystem_range, kPotentiometerOffset,
                                     climber_pot_translate_inverse_test, false,
                                     kLimitBuffer));
  }
}

TEST(WpilibUtilsTest, ReverseRange) {
  {
    // Physical pot range: 0.0 to 0.5, reversed: -0.5 to 0.0
    constexpr double kPotentiometerOffset = 0.0;
    constexpr double kLimitBuffer = 0.00;

    ::frc971::constants::Range subsystem_range;
    subsystem_range = ::frc971::constants::Range{
        .lower_hard = -0.49, .upper_hard = -0.01, .lower = -0.4, .upper = -0.1};
    EXPECT_TRUE(SafePotVoltageRange(subsystem_range, kPotentiometerOffset,
                                    climber_pot_translate_inverse_test, true,
                                    kLimitBuffer));
  }
  {
    // Physical pot range: 0.0 to 0.5
    constexpr double kPotentiometerOffset = 0.0;
    constexpr double kLimitBuffer = 0.00;

    ::frc971::constants::Range subsystem_range;
    subsystem_range = ::frc971::constants::Range{
        .lower_hard = -0.49, .upper_hard = -0.01, .lower = -0.4, .upper = -0.1};
    EXPECT_FALSE(SafePotVoltageRange(subsystem_range, kPotentiometerOffset,
                                     climber_pot_translate_inverse_test, false,
                                     kLimitBuffer));
  }
}

}  // namespace
}  // namespace testing
}  // namespace wpilib
}  // namespace frc971