#include "WPILib/DriverStation.h"
#include "WPILib/RobotBase.h"

namespace aos {
namespace crio {

// Designed for a subclass (that implements all of the pure virtual methods...)
// to be passed to START_ROBOT_CLASS (a WPILib macro) to start all of the code.
class ControlsManager : public RobotBase {
 public:
  ControlsManager();

  virtual void StartCompetition();

  static ControlsManager &GetInstance() {
    return *static_cast<ControlsManager *>(&RobotBase::getInstance());
  }
  DriverStation *GetDS() {
    return m_ds;
  }

 private:
  // Hooks that subclasses have to implement to do the correct things at the
  // correct times.
  virtual void CreateObjects() = 0;
  virtual void RegisterControlLoops() = 0;
  virtual void StartSensorBroadcasters() = 0;
};

}  // namespace crio
}  // namespace aos
