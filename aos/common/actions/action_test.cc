#include <unistd.h>

#include <memory>
#include <thread>
#include <chrono>

#include "gtest/gtest.h"

#include "aos/common/queue.h"
#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"
#include "aos/common/actions/actions.q.h"
#include "aos/common/actions/test_action.q.h"
#include "aos/common/event.h"
#include "aos/testing/test_shm.h"

namespace aos {
namespace common {
namespace actions {
namespace testing {


namespace chrono = ::std::chrono;

class TestActorIndex
    : public aos::common::actions::ActorBase<actions::TestActionQueueGroup> {
 public:
  explicit TestActorIndex(actions::TestActionQueueGroup *s)
      : aos::common::actions::ActorBase<actions::TestActionQueueGroup>(s) {}

  bool RunAction(const uint32_t &new_index) override {
    index = new_index;
    return true;
  }

  uint32_t index = 0;
};

::std::unique_ptr<
    aos::common::actions::TypedAction<actions::TestActionQueueGroup>>
MakeTestActionIndex(uint32_t index) {
  return ::std::unique_ptr<
      aos::common::actions::TypedAction<actions::TestActionQueueGroup>>(
      new aos::common::actions::TypedAction<actions::TestActionQueueGroup>(
          &actions::test_action, index));
}

class TestActorNOP
    : public aos::common::actions::ActorBase<actions::TestActionQueueGroup> {
 public:
  explicit TestActorNOP(actions::TestActionQueueGroup *s)
      : actions::ActorBase<actions::TestActionQueueGroup>(s) {}

  bool RunAction(const uint32_t &) override { return true; }
};

::std::unique_ptr<
    aos::common::actions::TypedAction<actions::TestActionQueueGroup>>
MakeTestActionNOP() {
  return MakeTestActionIndex(0);
}

class TestActorShouldCancel
    : public aos::common::actions::ActorBase<actions::TestActionQueueGroup> {
 public:
  explicit TestActorShouldCancel(actions::TestActionQueueGroup *s)
      : aos::common::actions::ActorBase<actions::TestActionQueueGroup>(s) {}

  bool RunAction(const uint32_t &) override {
    while (!ShouldCancel()) {
      LOG(FATAL, "NOT CANCELED!!\n");
    }
    return true;
  }
};

::std::unique_ptr<
    aos::common::actions::TypedAction<actions::TestActionQueueGroup>>
MakeTestActionShouldCancel() {
  return MakeTestActionIndex(0);
}

class TestActor2Nop
    : public aos::common::actions::ActorBase<actions::TestAction2QueueGroup> {
 public:
  explicit TestActor2Nop(actions::TestAction2QueueGroup *s)
      : actions::ActorBase<actions::TestAction2QueueGroup>(s) {}

  bool RunAction(const actions::MyParams &) { return true; }
};

::std::unique_ptr<
    aos::common::actions::TypedAction<actions::TestAction2QueueGroup>>
MakeTestAction2NOP(const actions::MyParams &params) {
  return ::std::unique_ptr<
      aos::common::actions::TypedAction<actions::TestAction2QueueGroup>>(
      new aos::common::actions::TypedAction<actions::TestAction2QueueGroup>(
          &actions::test_action2, params));
}

class ActionTest : public ::testing::Test {
 protected:
  ActionTest() {
    actions::test_action.goal.Clear();
    actions::test_action.status.Clear();
    actions::test_action2.goal.Clear();
    actions::test_action2.status.Clear();
  }

  virtual ~ActionTest() {
    actions::test_action.goal.Clear();
    actions::test_action.status.Clear();
    actions::test_action2.goal.Clear();
    actions::test_action2.status.Clear();
  }

  // Bring up and down Core.
  ::aos::testing::TestSharedMemory my_shm_;
  ::aos::common::actions::ActionQueue action_queue_;
};

// Tests that the the actions exist in a safe state at startup.
TEST_F(ActionTest, DoesNothing) {
  // Tick an empty queue and make sure it was not running.
  EXPECT_FALSE(action_queue_.Running());
  action_queue_.Tick();
  EXPECT_FALSE(action_queue_.Running());
}

// Tests that starting with an old run message in the goal queue actually works.
// This used to result in the client hanging, waiting for a response to its
// cancel message.
TEST_F(ActionTest, StartWithOldGoal) {
  ASSERT_TRUE(actions::test_action.goal.MakeWithBuilder().run(971).Send());

  TestActorNOP nop_act(&actions::test_action);

  ASSERT_FALSE(actions::test_action.status.FetchLatest());
  ::std::thread init_thread([&nop_act]() { nop_act.Initialize(); });
  ::std::this_thread::sleep_for(chrono::milliseconds(100));
  ASSERT_TRUE(actions::test_action.goal.MakeWithBuilder().run(1).Send());
  init_thread.join();
  ASSERT_TRUE(actions::test_action.status.FetchLatest());
  EXPECT_EQ(0u, actions::test_action.status->running);
  EXPECT_EQ(0u, actions::test_action.status->last_running);

  action_queue_.EnqueueAction(MakeTestActionNOP());
  nop_act.WaitForActionRequest();

  // We started an action and it should be running.
  EXPECT_TRUE(action_queue_.Running());

  action_queue_.CancelAllActions();
  action_queue_.Tick();

  EXPECT_TRUE(action_queue_.Running());

  // Run the action so it can signal completion.
  nop_act.RunIteration();
  action_queue_.Tick();

  // Make sure it stopped.
  EXPECT_FALSE(action_queue_.Running());
}

// Tests that the queues are properly configured for testing. Tests that queues
// work exactly as used in the tests.
TEST_F(ActionTest, QueueCheck) {
  actions::TestActionQueueGroup *send_side = &actions::test_action;
  actions::TestActionQueueGroup *recv_side = &actions::test_action;

  send_side->goal.MakeWithBuilder().run(1).Send();

  EXPECT_TRUE(recv_side->goal.FetchLatest());
  EXPECT_TRUE(recv_side->goal->run);

  send_side->goal.MakeWithBuilder().run(0).Send();

  EXPECT_TRUE(recv_side->goal.FetchLatest());
  EXPECT_FALSE(recv_side->goal->run);

  send_side->status.MakeWithBuilder().running(5).last_running(6).Send();

  EXPECT_TRUE(recv_side->status.FetchLatest());
  EXPECT_EQ(5, static_cast<int>(recv_side->status->running));
  EXPECT_EQ(6, static_cast<int>(recv_side->status->last_running));
}

// Tests that an action starts and stops.
TEST_F(ActionTest, ActionQueueWasRunning) {
  TestActorNOP nop_act(&actions::test_action);

  // Tick an empty queue and make sure it was not running.
  action_queue_.Tick();
  EXPECT_FALSE(action_queue_.Running());

  action_queue_.EnqueueAction(MakeTestActionNOP());
  nop_act.WaitForActionRequest();

  // We started an action and it should be running.
  EXPECT_TRUE(action_queue_.Running());

  // Tick it and make sure it is still running.
  action_queue_.Tick();
  EXPECT_TRUE(action_queue_.Running());

  // Run the action so it can signal completion.
  nop_act.RunIteration();
  action_queue_.Tick();

  // Make sure it stopped.
  EXPECT_FALSE(action_queue_.Running());
}

// Tests that we can cancel two actions and have them both stop.
TEST_F(ActionTest, ActionQueueCancelAll) {
  TestActorNOP nop_act(&actions::test_action);

  // Tick an empty queue and make sure it was not running.
  action_queue_.Tick();
  EXPECT_FALSE(action_queue_.Running());

  // Enqueue two actions to test both cancel. We can have an action and a next
  // action so we want to test that.
  action_queue_.EnqueueAction(MakeTestActionNOP());
  action_queue_.EnqueueAction(MakeTestActionNOP());
  nop_act.WaitForActionRequest();
  action_queue_.Tick();

  // Check that current and next exist.
  EXPECT_TRUE(action_queue_.GetCurrentActionState(nullptr, nullptr, nullptr,
                                                  nullptr, nullptr, nullptr));
  EXPECT_TRUE(action_queue_.GetNextActionState(nullptr, nullptr, nullptr,
                                               nullptr, nullptr, nullptr));

  action_queue_.CancelAllActions();
  action_queue_.Tick();

  // It should still be running as the actor could not have signaled.
  EXPECT_TRUE(action_queue_.Running());

  bool sent_started, sent_cancel, interrupted;
  EXPECT_TRUE(action_queue_.GetCurrentActionState(
      nullptr, &sent_started, &sent_cancel, &interrupted, nullptr, nullptr));
  EXPECT_TRUE(sent_started);
  EXPECT_TRUE(sent_cancel);
  EXPECT_FALSE(interrupted);

  EXPECT_FALSE(action_queue_.GetNextActionState(nullptr, nullptr, nullptr,
                                                nullptr, nullptr, nullptr));

  // Run the action so it can signal completion.
  nop_act.RunIteration();
  action_queue_.Tick();

  // Make sure it stopped.
  EXPECT_FALSE(action_queue_.Running());
}

// Tests that an action that would block forever stops when canceled.
TEST_F(ActionTest, ActionQueueCancelOne) {
  TestActorShouldCancel cancel_act(&actions::test_action);

  // Enqueue blocking action.
  action_queue_.EnqueueAction(MakeTestActionShouldCancel());

  cancel_act.WaitForActionRequest();
  action_queue_.Tick();
  EXPECT_TRUE(action_queue_.Running());

  // Tell action to cancel.
  action_queue_.CancelCurrentAction();
  action_queue_.Tick();

  // This will block forever on failure.
  // TODO(ben): prolly a bad way to fail
  cancel_act.RunIteration();
  action_queue_.Tick();

  // It should still be running as the actor could not have signalled.
  EXPECT_FALSE(action_queue_.Running());
}

// Tests that an action starts and stops.
TEST_F(ActionTest, ActionQueueTwoActions) {
  TestActorNOP nop_act(&actions::test_action);

  // Tick an empty queue and make sure it was not running.
  action_queue_.Tick();
  EXPECT_FALSE(action_queue_.Running());

  // Enqueue action to be canceled.
  action_queue_.EnqueueAction(MakeTestActionNOP());
  nop_act.WaitForActionRequest();
  action_queue_.Tick();

  // Should still be running as the actor could not have signalled.
  EXPECT_TRUE(action_queue_.Running());

  // id for the first time run.
  uint32_t nop_act_id = 0;
  // Check the internal state and write down id for later use.
  bool sent_started, sent_cancel, interrupted;
  EXPECT_TRUE(action_queue_.GetCurrentActionState(nullptr, &sent_started,
                                                  &sent_cancel, &interrupted,
                                                  &nop_act_id, nullptr));
  EXPECT_TRUE(sent_started);
  EXPECT_FALSE(sent_cancel);
  EXPECT_FALSE(interrupted);
  ASSERT_NE(0u, nop_act_id);

  // Add the next action which should ensure the first stopped.
  action_queue_.EnqueueAction(MakeTestActionNOP());

  // id for the second run.
  uint32_t nop_act2_id = 0;
  // Check the internal state and write down id for later use.
  EXPECT_TRUE(action_queue_.GetNextActionState(nullptr, &sent_started,
                                               &sent_cancel, &interrupted,
                                               &nop_act2_id, nullptr));
  EXPECT_NE(nop_act_id, nop_act2_id);
  EXPECT_FALSE(sent_started);
  EXPECT_FALSE(sent_cancel);
  EXPECT_FALSE(interrupted);
  ASSERT_NE(0u, nop_act2_id);

  action_queue_.Tick();

  // Run the action so it can signal completion.
  nop_act.RunIteration();
  action_queue_.Tick();
  // Wait for the first id to finish, needed for the correct number of fetches.
  nop_act.WaitForStop(nop_act_id);

  // Start the next action on the actor side.
  nop_act.WaitForActionRequest();

  // Check the new action is the right one.
  uint32_t test_id = 0;
  EXPECT_TRUE(action_queue_.GetCurrentActionState(
      nullptr, &sent_started, &sent_cancel, &interrupted, &test_id, nullptr));
  EXPECT_TRUE(sent_started);
  EXPECT_FALSE(sent_cancel);
  EXPECT_FALSE(interrupted);
  EXPECT_EQ(nop_act2_id, test_id);

  // Make sure it is still going.
  EXPECT_TRUE(action_queue_.Running());

  // Run the next action so it can accomplish signal completion.
  nop_act.RunIteration();
  action_queue_.Tick();
  nop_act.WaitForStop(nop_act_id);

  // Make sure it stopped.
  EXPECT_FALSE(action_queue_.Running());
}

// Tests that we do get an index with our goal
TEST_F(ActionTest, ActionIndex) {
  TestActorIndex idx_act(&actions::test_action);

  // Tick an empty queue and make sure it was not running.
  action_queue_.Tick();
  EXPECT_FALSE(action_queue_.Running());

  // Enqueue action to post index.
  action_queue_.EnqueueAction(MakeTestActionIndex(5));
  EXPECT_TRUE(actions::test_action.goal.FetchLatest());
  EXPECT_EQ(5u, actions::test_action.goal->params);
  EXPECT_EQ(0u, idx_act.index);

  idx_act.WaitForActionRequest();
  action_queue_.Tick();

  // Check the new action is the right one.
  uint32_t test_id = 0;
  EXPECT_TRUE(action_queue_.GetCurrentActionState(nullptr, nullptr, nullptr,
                                                  nullptr, &test_id, nullptr));

  // Run the next action so it can accomplish signal completion.
  idx_act.RunIteration();
  action_queue_.Tick();
  idx_act.WaitForStop(test_id);
  EXPECT_EQ(5u, idx_act.index);

  // Enqueue action to post index.
  action_queue_.EnqueueAction(MakeTestActionIndex(3));
  EXPECT_TRUE(actions::test_action.goal.FetchLatest());
  EXPECT_EQ(3u, actions::test_action.goal->params);

  // Run the next action so it can accomplish signal completion.
  idx_act.RunIteration();
  action_queue_.Tick();
  idx_act.WaitForStop(test_id);
  EXPECT_EQ(3u, idx_act.index);
}

// Tests that an action with a structure params works.
TEST_F(ActionTest, StructParamType) {
  TestActor2Nop nop_act(&actions::test_action2);

  // Tick an empty queue and make sure it was not running.
  action_queue_.Tick();
  EXPECT_FALSE(action_queue_.Running());

  actions::MyParams p;
  p.param1 = 5.0;
  p.param2 = 7;

  action_queue_.EnqueueAction(MakeTestAction2NOP(p));
  nop_act.WaitForActionRequest();

  // We started an action and it should be running.
  EXPECT_TRUE(action_queue_.Running());

  // Tick it and make sure it is still running.
  action_queue_.Tick();
  EXPECT_TRUE(action_queue_.Running());

  // Run the action so it can signal completion.
  nop_act.RunIteration();
  action_queue_.Tick();

  // Make sure it stopped.
  EXPECT_FALSE(action_queue_.Running());
}

// Tests that cancelling an action before the message confirming it started is
// received works.
// Situations like this used to lock the action queue up waiting for an action
// to report that it successfully cancelled.
// This situation is kind of a race condition, but it happens very consistently
// when hitting buttons while the robot is in teleop-disabled. To hit the race
// condition consistently in the test, there are a couple of Events inserted in
// between various things running.
TEST_F(ActionTest, CancelBeforeStart) {
  Event thread_ready, ready_to_start, ready_to_stop;
  ::std::thread action_thread(
      [this, &thread_ready, &ready_to_start, &ready_to_stop]() {
        TestActorNOP nop_act(&actions::test_action);
        nop_act.Initialize();
        thread_ready.Set();
        ready_to_start.Wait();
        nop_act.WaitForActionRequest();
        LOG(DEBUG, "got a request to run\n");
        const uint32_t running_id = nop_act.RunIteration();
        LOG(DEBUG, "waiting for %" PRIx32 " to be stopped\n", running_id);
        ready_to_stop.Set();
        nop_act.WaitForStop(running_id);
      });

  action_queue_.CancelAllActions();
  EXPECT_FALSE(action_queue_.Running());
  thread_ready.Wait();
  LOG(DEBUG, "starting action\n");
  action_queue_.EnqueueAction(MakeTestActionNOP());
  action_queue_.Tick();
  action_queue_.CancelAllActions();
  ready_to_start.Set();
  LOG(DEBUG, "started action\n");
  EXPECT_TRUE(action_queue_.Running());
  ready_to_stop.Wait();
  EXPECT_TRUE(action_queue_.Running());
  LOG(DEBUG, "action is ready to stop\n");

  action_queue_.Tick();
  action_queue_.CancelAllActions();
  EXPECT_FALSE(action_queue_.Running());
  action_queue_.Tick();
  action_queue_.CancelAllActions();
  ASSERT_FALSE(action_queue_.Running());
  action_thread.join();

  action_queue_.Tick();
  action_queue_.CancelAllActions();
  ASSERT_FALSE(action_queue_.Running());
}

}  // namespace testing
}  // namespace actions
}  // namespace common
}  // namespace aos
