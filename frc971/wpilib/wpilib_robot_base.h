#ifndef FRC971_WPILIB_NEWROBOTBASE_H_
#define FRC971_WPILIB_NEWROBOTBASE_H_

#ifdef WPILIB2015
#include "RobotBase.h"
#else
#include "SampleRobot.h"
#endif

namespace frc971 {
namespace wpilib {

class WPILibRobotBase {
public:
  virtual void Run() = 0;
};

#define AOS_ROBOT_CLASS(_ClassName_) \
  START_ROBOT_CLASS(::frc971::wpilib::WPILibAdapterRobot<_ClassName_>)

template <typename T>
#ifdef WPILIB2015
class WPILibAdapterRobot : public RobotBase {
#else
class WPILibAdapterRobot : public SampleRobot {
#endif
 public:
#ifdef WPILIB2015
  void StartCompetition() override { robot_.Run(); }
#else
  void RobotMain() override { robot_.Run(); }
#endif

 private:
  T robot_;
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_NEWROBOTBASE_H_
