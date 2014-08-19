#ifndef AOS_COMMON_CONTROLS_CONTROL_LOOP_TEST_H_
#define AOS_COMMON_CONTROLS_CONTROL_LOOP_TEST_H_

#include "gtest/gtest.h"

#include "aos/common/queue_testutils.h"
#include "aos/common/time.h"

namespace aos {
namespace testing {

// Handles setting up the environment that all control loops need to actually
// run.
// This includes sending the queue messages and Clear()ing the queues when
// appropriate.
// It also includes dealing with ::aos::time.
class ControlLoopTest : public ::testing::Test {
 public:
  ControlLoopTest();

  virtual ~ControlLoopTest();

  // Simulates everything that happens during 1 time step.
  void SimulateTimestep(bool enabled);

 private:
  bool sent_robot_state_last_time_ = false;
  ::aos::time::Time current_time_ = ::aos::time::Time::InSeconds(0);

  ::aos::common::testing::GlobalCoreInstance my_core;
};

}  // namespace testing
}  // namespace aos

#endif  // AOS_COMMON_CONTROLS_CONTROL_LOOP_TEST_H_
