#ifndef AOS_CRIO_MOTOR_SERVER_SENSOR_OUTPUT_H_
#define AOS_CRIO_MOTOR_SERVER_SENSOR_OUTPUT_H_

#include <semLib.h>
#include <vector>

namespace aos {

// Keeps track of instances of all instantiations.
class SensorOutputs {
 public:
  // Calls RunIteration on all instances and then runs all SensorInput
  // subclasses for that type.
  static void UpdateAll();
 private:
  static SEM_ID lock_;
  static std::vector<SensorOutputs *> outputs_running_;
 protected:
  // Calls RunIteration with a temporary Values instance and then runs all
  // SensorInput subclasses with the same Values type.
  virtual void Update() = 0;
};

// Class for implementing crio code that reads sensor values and puts them into
// the sensor struct.
template<class Values> class SensorOutput : public SensorOutputs {
 protected:
  // Fills out vals with the current data.
  // May not be called at anything close to consistent intervals and may be
  // called simultaneously with different arguments, so it must be reentrant.
  virtual void RunIteration(Values &vals) = 0;
 public:
  // Sets it up so that RunIteration will get called when appropriate.
  void Run();

  // Calls RunIteration on all instances with vals.
  static void RunIterationAll(Values &vals);
 private:
  static std::vector<SensorOutput<Values> *> output_running_;
  virtual void Update();
};

} // namespace aos

#include "SensorOutput-tmpl.h"

#endif

