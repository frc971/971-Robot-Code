#include "WPILib/DriverStation.h"
#include "WPILib/RobotBase.h"

namespace aos {
namespace crio {

class ControlsManager : public RobotBase {
 public:
  // Gets called when it is time to register all the control loops.
  virtual void RegisterControlLoops() = 0;
  virtual void StartCompetition();
  static inline ControlsManager &GetInstance() {
    return *static_cast<ControlsManager *>(&RobotBase::getInstance());
  }
  inline DriverStation *GetDS() {
    return m_ds;
  }
};

}  // namespace crio
}  // namespace aos
