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
  // Called when it is time to create anything that uses WPILib.
  virtual void CreateObjects() = 0;
  // Called when it is time to add controls loops to any CRIOControlLoopRunners.
  virtual void RegisterControlLoops() = 0;
  // Called when it is time to start any SensorBroadcasters.
  virtual void StartSensorBroadcasters() = 0;
};

}  // namespace crio
}  // namespace aos
