#ifndef FRC971_WPILIB_NEWROBOTBASE_H_
#define FRC971_WPILIB_NEWROBOTBASE_H_

#include "RobotBase.h"

namespace frc971 {
namespace wpilib {

class WPILibRobotBase {
public:
  virtual void Run() = 0;
};

#define AOS_ROBOT_CLASS(_ClassName_) \
  START_ROBOT_CLASS(::frc971::wpilib::WPILibAdapterRobot<_ClassName_>)

template <typename T>
class WPILibAdapterRobot : public RobotBase {
 public:
  void StartCompetition() override { robot_.Run(); }

 private:
  T robot_;
};

}
}

#endif // FRC971_WPILIB_NEWROBOTBASE_H_

