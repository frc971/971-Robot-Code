#ifndef AOS_INPUT_SENSOR_INPUT_H_
#define AOS_INPUT_SENSOR_INPUT_H_

#ifdef __VXWORKS__
#include <vector>
#include <semLib.h>
#endif

namespace aos {

// Class for implementing code that takes information from a sensor struct and
// places it into queues. Subclasses should be compiled for both the atom and
// the crio to support crio control loops.
template<class Values> class SensorInput {
 protected:
  virtual void RunIteration(Values &values) = 0;
 public:
  // Enters an infinite loop that reads values and calls RunIteration.
  void Run();

#ifdef __VXWORKS__
  // Calls RunIteration on all instances with values.
  static void RunIterationAll(Values &values);
 private:
  static SEM_ID lock_;
  static std::vector<SensorInput *> running_;
#endif
};

} // namespace aos

#include "SensorInput-tmpl.h"

#endif

