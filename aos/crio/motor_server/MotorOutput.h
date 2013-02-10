#ifndef AOS_CRIO_MOTOR_SERVER_MOTOR_OUTPUT_H_
#define AOS_CRIO_MOTOR_SERVER_MOTOR_OUTPUT_H_

#include <vector>
#include <semLib.h>

namespace aos {

// The place where the outputs from crio control loops get written out to the
// motors.
class MotorOutput {
 public:
  // Call RunIteration on all instances that have been Run.
  static void RunIterationAll();
  void Run();
 protected:
  // Write the outputs from crio control loops to wherever they go.
  virtual void RunIteration() = 0;
 private:
  static std::vector<MotorOutput *> instances;
  static SEM_ID lock;
};

} // namespace aos

#endif

