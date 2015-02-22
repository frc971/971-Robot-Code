#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "aos/common/actions/actor.h"
#include "frc971/actors/fridge_profile_action.q.h"
#include "frc971/actors/fridge_profile_actor.h"
#include "frc971/control_loops/fridge/fridge.q.h"

using ::aos::time::Time;

namespace frc971 {
namespace actors {
namespace testing {

class FridgeProfileTest : public ::testing::Test {
 protected:
  FridgeProfileTest() {
  frc971::actors::fridge_profile_action.goal.Clear();
  frc971::actors::fridge_profile_action.status.Clear();
  control_loops::fridge_queue.status.Clear();
  control_loops::fridge_queue.goal.Clear();
  }

  virtual ~FridgeProfileTest() {
  frc971::actors::fridge_profile_action.goal.Clear();
  frc971::actors::fridge_profile_action.status.Clear();
  control_loops::fridge_queue.status.Clear();
  control_loops::fridge_queue.goal.Clear();
  }

  // Bring up and down Core.
  ::aos::common::testing::GlobalCoreInstance my_core;
};

void GetVelAccel(double new_val, double* last_two, double* vel, double* accel) {
  *vel = new_val - last_two[0];
  *accel = (*vel) - (last_two[0] - last_two[1]);
  last_two[1] = last_two[0];
  last_two[0] = new_val;
}

// A very long manual test that checks every step of a profile given ridiculous
// values that generate a simple profile. Note that next_*_vel is the predicted
// velocity for the step (in m/s), while *_vel is the observed velocity of the
// last step (in m/step).
TEST_F(FridgeProfileTest, ProfileValid) {
  FridgeProfileActor fridge_profile(&frc971::actors::fridge_profile_action);
  EXPECT_TRUE(fridge_profile.InitializeProfile(200.0, 20000.0, 200.0, 20000.0));
  double last_angle[2] = {0.0, 0.0};
  double last_height[2] = {0.0, 0.0};
  double angle_vel = 0, angle_accel = 0, height_vel = 0, height_accel = 0;
  double next_angle = 0, next_height = 0, next_angle_vel = 0.0,
         next_height_vel = 0.0;

  // Angle (0.250000, 0.250000, 0.25) Height (0.250000, 0.250000, 0.25)
  EXPECT_TRUE(fridge_profile.IterateProfile(5.0, 5.0, &next_angle, &next_height,
                                            &next_angle_vel, &next_height_vel));
  GetVelAccel(next_angle, last_angle, &angle_vel, &angle_accel);
  GetVelAccel(next_height, last_height, &height_vel, &height_accel);
  EXPECT_EQ(0.25, next_angle);
  EXPECT_EQ(100.0, next_angle_vel);
  EXPECT_EQ(0.25, angle_vel);
  EXPECT_EQ(0.25, angle_accel);
  EXPECT_EQ(0.25, next_height);
  EXPECT_EQ(100.0, next_height_vel);
  EXPECT_EQ(0.25, height_vel);
  EXPECT_EQ(0.25, height_accel);

  // Angle (1.000000, 0.750000, 0.50) Height (1.000000, 0.750000, 0.50)
  EXPECT_TRUE(fridge_profile.IterateProfile(5.0, 5.0, &next_angle, &next_height,
                                            &next_angle_vel, &next_height_vel));
  GetVelAccel(next_angle, last_angle, &angle_vel, &angle_accel);
  GetVelAccel(next_height, last_height, &height_vel, &height_accel);
  EXPECT_EQ(1.0, next_angle);
  EXPECT_EQ(200.0, next_angle_vel);
  EXPECT_EQ(0.75, angle_vel);
  EXPECT_EQ(0.50, angle_accel);
  EXPECT_EQ(1.0, next_height);
  EXPECT_EQ(200.0, next_height_vel);
  EXPECT_EQ(0.75, height_vel);
  EXPECT_EQ(0.50, height_accel);

  // Angle (2.000000, 1.000000, 0.25) Height (2.000000, 1.000000, 0.25)
  EXPECT_TRUE(fridge_profile.IterateProfile(5.0, 5.0, &next_angle, &next_height,
                                            &next_angle_vel, &next_height_vel));
  GetVelAccel(next_angle, last_angle, &angle_vel, &angle_accel);
  GetVelAccel(next_height, last_height, &height_vel, &height_accel);
  EXPECT_EQ(2.0, next_angle);
  EXPECT_EQ(200.0, next_angle_vel);
  EXPECT_EQ(1.0, angle_vel);
  EXPECT_EQ(0.25, angle_accel);
  EXPECT_EQ(2.0, next_height);
  EXPECT_EQ(200.0, next_height_vel);
  EXPECT_EQ(1.0, height_vel);
  EXPECT_EQ(0.25, height_accel);

  // Angle (3.000000, 1.000000, 0.00) Height (3.000000, 1.000000, 0.00)
  EXPECT_TRUE(fridge_profile.IterateProfile(5.0, 5.0, &next_angle, &next_height,
                                            &next_angle_vel, &next_height_vel));
  GetVelAccel(next_angle, last_angle, &angle_vel, &angle_accel);
  GetVelAccel(next_height, last_height, &height_vel, &height_accel);
  EXPECT_EQ(3.0, next_angle);
  EXPECT_EQ(200.0, next_angle_vel);
  EXPECT_EQ(1.0, angle_vel);
  EXPECT_EQ(0.0, angle_accel);
  EXPECT_EQ(3.0, next_height);
  EXPECT_EQ(200.0, next_height_vel);
  EXPECT_EQ(1.0, height_vel);
  EXPECT_EQ(0.0, height_accel);

  // Angle (4.000000, 1.000000, 0.00) Height (4.000000, 1.000000, 0.00)
  EXPECT_TRUE(fridge_profile.IterateProfile(5.0, 5.0, &next_angle, &next_height,
                                            &next_angle_vel, &next_height_vel));
  GetVelAccel(next_angle, last_angle, &angle_vel, &angle_accel);
  GetVelAccel(next_height, last_height, &height_vel, &height_accel);
  EXPECT_EQ(4.0, next_angle);
  EXPECT_EQ(200.0, next_angle_vel);
  EXPECT_EQ(1.0, angle_vel);
  EXPECT_EQ(0.0, angle_accel);
  EXPECT_EQ(4.0, next_height);
  EXPECT_EQ(200.0, next_height_vel);
  EXPECT_EQ(1.0, height_vel);
  EXPECT_EQ(0.0, height_accel);

  // Angle (4.750000, 0.750000, -0.25) Height (4.750000, 0.750000, -0.25)
  EXPECT_TRUE(fridge_profile.IterateProfile(5.0, 5.0, &next_angle, &next_height,
                                            &next_angle_vel, &next_height_vel));
  GetVelAccel(next_angle, last_angle, &angle_vel, &angle_accel);
  GetVelAccel(next_height, last_height, &height_vel, &height_accel);
  EXPECT_EQ(4.75, next_angle);
  EXPECT_EQ(100.0, next_angle_vel);
  EXPECT_EQ(0.75, angle_vel);
  EXPECT_EQ(-0.25, angle_accel);
  EXPECT_EQ(4.75, next_height);
  EXPECT_EQ(100.0, next_height_vel);
  EXPECT_EQ(0.75, height_vel);
  EXPECT_EQ(-0.25, height_accel);

  // Angle (5.000000, 0.250000, -0.50) Height (5.000000, 0.250000, -0.50)
  EXPECT_TRUE(fridge_profile.IterateProfile(5.0, 5.0, &next_angle, &next_height,
                                            &next_angle_vel, &next_height_vel));
  GetVelAccel(next_angle, last_angle, &angle_vel, &angle_accel);
  GetVelAccel(next_height, last_height, &height_vel, &height_accel);
  EXPECT_EQ(5.0, next_angle);
  EXPECT_EQ(0.0, next_angle_vel);
  EXPECT_EQ(0.25, angle_vel);
  EXPECT_EQ(-0.50, angle_accel);
  EXPECT_EQ(5.0, next_height);
  EXPECT_EQ(0.0, next_height_vel);
  EXPECT_EQ(0.25, height_vel);
  EXPECT_EQ(-0.50, height_accel);

  // Angle (5.000000, 0.000000, -0.25) Height (5.000000, 0.000000, -0.25)
  EXPECT_TRUE(fridge_profile.IterateProfile(5.0, 5.0, &next_angle, &next_height,
                                            &next_angle_vel, &next_height_vel));
  GetVelAccel(next_angle, last_angle, &angle_vel, &angle_accel);
  GetVelAccel(next_height, last_height, &height_vel, &height_accel);
  EXPECT_EQ(5.0, next_angle);
  EXPECT_EQ(0.0, next_angle_vel);
  EXPECT_EQ(0.0, angle_vel);
  EXPECT_EQ(-0.25, angle_accel);
  EXPECT_EQ(5.0, next_height);
  EXPECT_EQ(0.0, next_height_vel);
  EXPECT_EQ(0.0, height_vel);
  EXPECT_EQ(-0.25, height_accel);

  // Angle (5.000000, 0.000000, 0.00) Height (5.000000, 0.000000, 0.00)
  EXPECT_TRUE(fridge_profile.IterateProfile(5.0, 5.0, &next_angle, &next_height,
                                            &next_angle_vel, &next_height_vel));
  GetVelAccel(next_angle, last_angle, &angle_vel, &angle_accel);
  GetVelAccel(next_height, last_height, &height_vel, &height_accel);
  EXPECT_EQ(5.0, next_angle);
  EXPECT_EQ(0.0, next_angle_vel);
  EXPECT_EQ(0.0, angle_vel);
  EXPECT_EQ(0.0, angle_accel);
  EXPECT_EQ(5.0, next_height);
  EXPECT_EQ(0.0, next_height_vel);
  EXPECT_EQ(0.0, height_vel);
  EXPECT_EQ(0.0, height_accel);
}

// Tests that we get to our first goal, then change the goal and get there under
// constraints.
TEST_F(FridgeProfileTest, ProfileChangeGoal) {
  FridgeProfileActor fridge_profile(&frc971::actors::fridge_profile_action);
  EXPECT_TRUE(fridge_profile.InitializeProfile(200.0, 20000.0, 200.0, 20000.0));
  double last_angle[2] = {0.0, 0.0};
  double last_height[2] = {0.0, 0.0};
  double angle_vel = 0, angle_accel = 0, height_vel = 0, height_accel = 0;
  double next_angle = 0, next_height = 0, next_angle_vel = 0.0,
         next_height_vel = 0.0;

  for (int i = 0; i < 7; i++) {
    EXPECT_TRUE(fridge_profile.IterateProfile(5.0, 5.0, &next_angle,
                                              &next_height, &next_angle_vel,
                                              &next_height_vel));
    GetVelAccel(next_angle, last_angle, &angle_vel, &angle_accel);
    GetVelAccel(next_height, last_height, &height_vel, &height_accel);
    EXPECT_GE(1.0, angle_vel);
    EXPECT_GE(0.5, angle_accel);
    EXPECT_LE(-1.0, angle_vel);
    EXPECT_LE(-0.5, angle_accel);
  }

  EXPECT_EQ(5.0, next_angle);

  for (int i = 0; i < 7; i++) {
    EXPECT_TRUE(fridge_profile.IterateProfile(10.0, 10.0, &next_angle,
                                              &next_height, &next_angle_vel,
                                              &next_height_vel));
    GetVelAccel(next_angle, last_angle, &angle_vel, &angle_accel);
    GetVelAccel(next_height, last_height, &height_vel, &height_accel);
    EXPECT_GE(1.0, angle_vel);
    EXPECT_GE(0.5, angle_accel);
    EXPECT_LE(-1.0, angle_vel);
    EXPECT_LE(-0.5, angle_accel);
  }

  EXPECT_EQ(10.0, next_angle);
}

// Use our simple little profile with a queue to check we get the same result
TEST_F(FridgeProfileTest, ProfileQueueValid) {
  FridgeProfileActor fridge_profile(&frc971::actors::fridge_profile_action);

  FridgeProfileParams params;
  params.arm_angle = 5.0;
  params.arm_max_velocity = 200.0;
  params.arm_max_acceleration = 20000.0;
  params.elevator_height = 5.0;
  params.elevator_max_velocity = 200.0;
  params.elevator_max_acceleration = 20000.0;
  params.top_front_grabber = true;
  params.top_back_grabber = false;
  params.bottom_front_grabber = true;
  params.bottom_back_grabber = false;

  frc971::actors::fridge_profile_action.goal.MakeWithBuilder()
      .run(true)
      .params(params)
      .Send();

  // tell it the fridge is zeroed
  control_loops::fridge_queue.status.MakeWithBuilder()
      .zeroed(true)
      .angle(0.0)
      .height(0.0)
      .Send();

  fridge_profile.SetTesting();

  // do the action and it will post to the goal queue
  fridge_profile.WaitForActionRequest();
  fridge_profile.RunAction(params);

  // a= 0.250000, e= 0.250000, av= 100.000000, ev= 100.000000
  EXPECT_TRUE(control_loops::fridge_queue.goal.FetchNext());
  EXPECT_TRUE(control_loops::fridge_queue.goal.get());
  EXPECT_EQ(0.25, control_loops::fridge_queue.goal->angle);
  EXPECT_EQ(0.25, control_loops::fridge_queue.goal->height);
  EXPECT_EQ(100.0, control_loops::fridge_queue.goal->angular_velocity);
  EXPECT_EQ(100.0, control_loops::fridge_queue.goal->velocity);

  // a= 1.000000, e= 1.000000, av= 200.000000, ev= 200.000000
  EXPECT_TRUE(control_loops::fridge_queue.goal.FetchNext());
  EXPECT_TRUE(control_loops::fridge_queue.goal.get());
  EXPECT_EQ(1.0, control_loops::fridge_queue.goal->angle);
  EXPECT_EQ(1.0, control_loops::fridge_queue.goal->height);
  EXPECT_EQ(200.0, control_loops::fridge_queue.goal->angular_velocity);
  EXPECT_EQ(200.0, control_loops::fridge_queue.goal->velocity);

  // a= 2.000000, e= 2.000000, av= 200.000000, ev= 200.000000
  EXPECT_TRUE(control_loops::fridge_queue.goal.FetchNext());
  EXPECT_TRUE(control_loops::fridge_queue.goal.get());
  EXPECT_EQ(2.0, control_loops::fridge_queue.goal->angle);
  EXPECT_EQ(2.0, control_loops::fridge_queue.goal->height);
  EXPECT_EQ(200.0, control_loops::fridge_queue.goal->angular_velocity);
  EXPECT_EQ(200.0, control_loops::fridge_queue.goal->velocity);

  // a= 3.000000, e= 3.000000, av= 200.000000, ev= 200.000000
  EXPECT_TRUE(control_loops::fridge_queue.goal.FetchNext());
  EXPECT_TRUE(control_loops::fridge_queue.goal.get());
  EXPECT_EQ(3.0, control_loops::fridge_queue.goal->angle);
  EXPECT_EQ(3.0, control_loops::fridge_queue.goal->height);
  EXPECT_EQ(200.0, control_loops::fridge_queue.goal->angular_velocity);
  EXPECT_EQ(200.0, control_loops::fridge_queue.goal->velocity);

  // a= 4.000000, e= 4.000000, av= 200.000000, ev= 200.000000
  EXPECT_TRUE(control_loops::fridge_queue.goal.FetchNext());
  EXPECT_TRUE(control_loops::fridge_queue.goal.get());
  EXPECT_EQ(4.0, control_loops::fridge_queue.goal->angle);
  EXPECT_EQ(4.0, control_loops::fridge_queue.goal->height);
  EXPECT_EQ(200.0, control_loops::fridge_queue.goal->angular_velocity);
  EXPECT_EQ(200.0, control_loops::fridge_queue.goal->velocity);

  // a= 4.750000, e= 4.750000, av= 100.000000, ev= 100.000000
  EXPECT_TRUE(control_loops::fridge_queue.goal.FetchNext());
  EXPECT_TRUE(control_loops::fridge_queue.goal.get());
  EXPECT_EQ(4.75, control_loops::fridge_queue.goal->angle);
  EXPECT_EQ(4.75, control_loops::fridge_queue.goal->height);
  EXPECT_EQ(100.0, control_loops::fridge_queue.goal->angular_velocity);
  EXPECT_EQ(100.0, control_loops::fridge_queue.goal->velocity);

  // a= 5.000000, e= 5.000000, av= 0.000000, ev= 0.000000
  EXPECT_TRUE(control_loops::fridge_queue.goal.FetchNext());
  EXPECT_TRUE(control_loops::fridge_queue.goal.get());
  EXPECT_EQ(5.0, control_loops::fridge_queue.goal->angle);
  EXPECT_EQ(5.0, control_loops::fridge_queue.goal->height);
  EXPECT_EQ(0.0, control_loops::fridge_queue.goal->angular_velocity);
  EXPECT_EQ(0.0, control_loops::fridge_queue.goal->velocity);

  // that should be all
  EXPECT_FALSE(control_loops::fridge_queue.goal.FetchNext());
}

// Make sure that giving 0 velocity+acceleration makes it not move at all.
TEST_F(FridgeProfileTest, ProfileNoVel) {
  FridgeProfileActor fridge_profile(&frc971::actors::fridge_profile_action);

  FridgeProfileParams params;
  params.arm_angle = 5.0;
  params.arm_max_velocity = 200.0;
  params.arm_max_acceleration = 20000.0;
  params.elevator_height = 5.0;
  params.elevator_max_velocity = 0.0;
  params.elevator_max_acceleration = 0.0;
  params.top_front_grabber = true;
  params.top_back_grabber = false;
  params.bottom_front_grabber = true;
  params.bottom_back_grabber = false;

  frc971::actors::fridge_profile_action.goal.MakeWithBuilder()
      .run(true)
      .params(params)
      .Send();

  // tell it the fridge is zeroed
  control_loops::fridge_queue.status.MakeWithBuilder()
      .zeroed(true)
      .angle(0.0)
      .height(0.0)
      .Send();

  fridge_profile.SetTesting();

  // do the action and it will post to the goal queue
  fridge_profile.WaitForActionRequest();
  fridge_profile.RunAction(params);

  while (control_loops::fridge_queue.goal.FetchNext()) {
    EXPECT_EQ(0.0, control_loops::fridge_queue.goal->height);
  }
}

// Make sure that should cancel gets set by pushing to queue.
TEST_F(FridgeProfileTest, ProfileShouldCancel) {
  FridgeProfileActor fridge_profile(&frc971::actors::fridge_profile_action);
  double next_angle = 0, next_height = 0, next_angle_vel = 0.0,
         next_height_vel = 0.0;
  FridgeProfileParams params;
  params.arm_angle = 5.0;
  params.arm_max_velocity = 200.0;
  params.arm_max_acceleration = 20000.0;
  frc971::actors::fridge_profile_action.goal.MakeWithBuilder()
      .run(true)
      .params(params)
      .Send();

  // tell it the fridge is zeroed
  control_loops::fridge_queue.status.MakeWithBuilder()
      .zeroed(true)
      .angle(0.0)
      .height(0.0)
      .Send();

  fridge_profile.SetTesting();

  fridge_profile.WaitForActionRequest();

  // Angle (0.250000, 0.250000, 0.25) Height (0.250000, 0.250000, 0.25)
  EXPECT_TRUE(fridge_profile.IterateProfile(5.0, 5.0, &next_angle, &next_height,
                                            &next_angle_vel, &next_height_vel));
  EXPECT_FALSE(fridge_profile.ShouldCancel());
  frc971::actors::fridge_profile_action.goal.MakeWithBuilder()
      .run(false)
      .params(params)
      .Send();

  EXPECT_TRUE(fridge_profile.ShouldCancel());

  // Angle (0.250000, 0.250000, 0.25) Height (0.250000, 0.250000, 0.25)
  EXPECT_TRUE(fridge_profile.IterateProfile(5.0, 5.0, &next_angle, &next_height,
                                            &next_angle_vel, &next_height_vel));
}

}  // namespace testing.
}  // namespace actors.
}  // namespace frc971.
