#include <unistd.h>

#include <memory>
#include <thread>
#include <chrono>

#include "gtest/gtest.h"

#include "aos/actions/actions.h"
#include "aos/actions/actions_generated.h"
#include "aos/actions/actor.h"
#include "aos/actions/test_action_generated.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/testing/test_logging.h"
#include "aos/testing/test_shm.h"

namespace aos {
namespace common {
namespace actions {
namespace testing {

namespace chrono = ::std::chrono;

class TestActorIndex
    : public aos::common::actions::ActorBase<actions::TestActionGoal> {
 public:
  typedef TypedActionFactory<actions::TestActionGoal> Factory;

  explicit TestActorIndex(::aos::EventLoop *event_loop)
      : aos::common::actions::ActorBase<actions::TestActionGoal>(
            event_loop, "/test_action") {}

  static Factory MakeFactory(::aos::EventLoop *event_loop) {
    return Factory(event_loop, "/test_action");
  }

  bool RunAction(const UInt *new_index) override {
    VLOG(1) << "New index " << FlatbufferToJson(new_index);
    index = new_index->val();
    return true;
  }

  uint32_t index = 0;
};

class TestActorNOP
    : public aos::common::actions::ActorBase<actions::TestActionGoal> {
 public:
  typedef TypedActionFactory<actions::TestActionGoal> Factory;

  explicit TestActorNOP(::aos::EventLoop *event_loop)
      : actions::ActorBase<actions::TestActionGoal>(
            event_loop, "/test_action") {}

  static Factory MakeFactory(::aos::EventLoop *event_loop) {
    return Factory(event_loop, "/test_action");
  }

  bool RunAction(const UInt *) override { return true; }
};

class TestActorShouldCancel
    : public aos::common::actions::ActorBase<actions::TestActionGoal> {
 public:
  typedef TypedActionFactory<actions::TestActionGoal> Factory;

  explicit TestActorShouldCancel(::aos::EventLoop *event_loop)
      : aos::common::actions::ActorBase<actions::TestActionGoal>(
            event_loop, "/test_action") {}

  static Factory MakeFactory(::aos::EventLoop *event_loop) {
    return Factory(event_loop, "/test_action");
  }

  bool RunAction(const UInt *) override {
    while (!ShouldCancel()) {
      AOS_LOG(FATAL, "NOT CANCELED!!\n");
    }
    return true;
  }
};

class TestActor2Nop
    : public aos::common::actions::ActorBase<actions::TestAction2Goal> {
 public:
  typedef TypedActionFactory<actions::TestAction2Goal> Factory;

  explicit TestActor2Nop(::aos::EventLoop *event_loop)
      : actions::ActorBase<actions::TestAction2Goal>(
            event_loop, "/test_action2") {}

  static Factory MakeFactory(::aos::EventLoop *event_loop) {
    return Factory(event_loop, "/test_action2");
  }

  bool RunAction(const actions::MyParams *) { return true; }
};

class ActionTest : public ::testing::Test {
 protected:
  ActionTest()
      : configuration_(
            configuration::ReadConfig("aos/actions/action_test_config.json")),
        event_loop_factory_(&configuration_.message()),
        actor1_event_loop_(event_loop_factory_.MakeEventLoop("actor1")),
        actor2_event_loop_(event_loop_factory_.MakeEventLoop("actor2")),
        test_event_loop_(event_loop_factory_.MakeEventLoop("test")) {
    ::aos::testing::EnableTestLogging();
  }

  FlatbufferDetachedBuffer<Configuration> configuration_;

  // Bring up and down Core.
  ::aos::SimulatedEventLoopFactory event_loop_factory_;

  ::std::unique_ptr<::aos::EventLoop> actor1_event_loop_;
  ::std::unique_ptr<::aos::EventLoop> actor2_event_loop_;
  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
};

// Tests that the the actions exist in a safe state at startup.
TEST_F(ActionTest, DoesNothing) {
  ActionQueue action_queue;
  // Tick an empty queue and make sure it was not running.
  EXPECT_FALSE(action_queue.Running());
  action_queue.Tick();
  EXPECT_FALSE(action_queue.Running());
}

// Tests that starting with an old run message in the goal queue actually works.
// This used to result in the client hanging, waiting for a response to its
// cancel message.
TEST_F(ActionTest, StartWithOldGoal) {
  ::std::unique_ptr<::aos::EventLoop> test2_event_loop =
      event_loop_factory_.MakeEventLoop("test2");
  ::aos::Sender<TestActionGoal> goal_sender =
      test2_event_loop->MakeSender<TestActionGoal>("/test_action");
  ::aos::Fetcher<Status> status_fetcher =
      test2_event_loop->MakeFetcher<Status>("/test_action");

  TestActorIndex::Factory nop_actor_factory =
      TestActorNOP::MakeFactory(test_event_loop_.get());

  ActionQueue action_queue;

  {
    ::aos::Sender<TestActionGoal>::Builder builder =
        goal_sender.MakeBuilder();

    TestActionGoal::Builder goal_builder =
        builder.MakeBuilder<TestActionGoal>();

    goal_builder.add_run(971);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  TestActorNOP nop_act(actor1_event_loop_.get());

  ASSERT_FALSE(status_fetcher.Fetch());

  event_loop_factory_.RunFor(chrono::seconds(1));

  ASSERT_TRUE(status_fetcher.Fetch());
  EXPECT_EQ(0u, status_fetcher->running());
  EXPECT_EQ(0u, status_fetcher->last_running());

  {
    UIntT uint;
    uint.val = 0;
    action_queue.EnqueueAction(nop_actor_factory.Make(uint));
  }

  // We started an action and it should be running.
  EXPECT_TRUE(action_queue.Running());

  action_queue.CancelAllActions();
  action_queue.Tick();

  EXPECT_TRUE(action_queue.Running());

  // Run the action so it can signal completion.
  event_loop_factory_.RunFor(chrono::seconds(1));
  action_queue.Tick();

  // Make sure it stopped.
  EXPECT_FALSE(action_queue.Running());
}

// Tests that an action starts and stops.
TEST_F(ActionTest, ActionQueueWasRunning) {
  TestActorNOP nop_act(actor1_event_loop_.get());

  TestActorIndex::Factory nop_actor_factory =
      TestActorNOP::MakeFactory(test_event_loop_.get());

  ActionQueue action_queue;

  // Tick an empty queue and make sure it was not running.
  event_loop_factory_.RunFor(chrono::seconds(1));
  action_queue.Tick();
  EXPECT_FALSE(action_queue.Running());

  {
    UIntT uint;
    uint.val = 0;
    action_queue.EnqueueAction(nop_actor_factory.Make(uint));
  }

  // We started an action and it should be running.
  EXPECT_TRUE(action_queue.Running());

  // Tick it and make sure it is still running.
  action_queue.Tick();
  EXPECT_TRUE(action_queue.Running());

  // Run the action so it can signal completion.
  event_loop_factory_.RunFor(chrono::seconds(1));
  action_queue.Tick();

  // Make sure it stopped.
  EXPECT_FALSE(action_queue.Running());
}

// Tests that we can cancel two actions and have them both stop.
TEST_F(ActionTest, ActionQueueCancelAll) {
  TestActorNOP nop_act(actor1_event_loop_.get());

  TestActorIndex::Factory nop_actor_factory =
      TestActorNOP::MakeFactory(test_event_loop_.get());

  ActionQueue action_queue;

  // Let the actor and action queue start up and confirm nothing is running.
  event_loop_factory_.RunFor(chrono::seconds(1));
  action_queue.Tick();

  EXPECT_FALSE(action_queue.Running());

  // Enqueue two actions to test both cancel. We can have an action and a next
  // action so we want to test that.
  {
    UIntT uint;
    uint.val = 0;
    action_queue.EnqueueAction(nop_actor_factory.Make(uint));
    action_queue.EnqueueAction(nop_actor_factory.Make(uint));
  }

  action_queue.Tick();

  // Check that current and next exist.
  EXPECT_TRUE(action_queue.GetCurrentActionState(nullptr, nullptr, nullptr,
                                                 nullptr, nullptr, nullptr));
  EXPECT_TRUE(action_queue.GetNextActionState(nullptr, nullptr, nullptr,
                                              nullptr, nullptr, nullptr));

  action_queue.CancelAllActions();
  action_queue.Tick();

  // It should still be running as the actor could not have signaled.
  EXPECT_TRUE(action_queue.Running());

  bool sent_started, sent_cancel, interrupted;
  EXPECT_TRUE(action_queue.GetCurrentActionState(
      nullptr, &sent_started, &sent_cancel, &interrupted, nullptr, nullptr));
  EXPECT_TRUE(sent_started);
  EXPECT_TRUE(sent_cancel);
  EXPECT_FALSE(interrupted);

  EXPECT_FALSE(action_queue.GetNextActionState(nullptr, nullptr, nullptr,
                                               nullptr, nullptr, nullptr));

  // Run the action so it can signal completion.
  event_loop_factory_.RunFor(chrono::seconds(1));

  action_queue.Tick();

  // Make sure it stopped.
  EXPECT_FALSE(action_queue.Running());
  EXPECT_EQ(1, nop_act.running_count());
}

// Tests that an action that would block forever stops when canceled.
TEST_F(ActionTest, ActionQueueCancelOne) {
  TestActorShouldCancel cancel_act(actor1_event_loop_.get());

  TestActorShouldCancel::Factory cancel_action_factory =
      TestActorShouldCancel::MakeFactory(test_event_loop_.get());

  ActionQueue action_queue;

  // Let the actor and action queue start up.
  event_loop_factory_.RunFor(chrono::seconds(1));
  action_queue.Tick();

  // Enqueue blocking action.
  {
    UIntT uint;
    uint.val = 0;
    action_queue.EnqueueAction(cancel_action_factory.Make(uint));
  }

  action_queue.Tick();
  EXPECT_TRUE(action_queue.Running());

  // Tell action to cancel.
  action_queue.CancelCurrentAction();
  action_queue.Tick();

  // This will block forever on failure.
  // TODO(ben): prolly a bad way to fail
  event_loop_factory_.RunFor(chrono::seconds(1));
  action_queue.Tick();

  // It should still be running as the actor could not have signalled.
  EXPECT_FALSE(action_queue.Running());
}

// Tests that 2 actions in a row causes the second one to cancel the first one.
TEST_F(ActionTest, ActionQueueTwoActions) {
  TestActorNOP nop_actor(actor1_event_loop_.get());

  TestActorIndex::Factory nop_actor_factory =
      TestActorNOP::MakeFactory(test_event_loop_.get());

  ActionQueue action_queue;
  // Tick an empty queue and make sure it was not running.
  event_loop_factory_.RunFor(chrono::seconds(1));
  action_queue.Tick();
  EXPECT_FALSE(action_queue.Running());

  // Enqueue action to be canceled.
  {
    UIntT uint;
    uint.val = 0;
    action_queue.EnqueueAction(nop_actor_factory.Make(uint));
  }
  action_queue.Tick();

  // Should still be running as the actor could not have signalled.
  EXPECT_TRUE(action_queue.Running());

  // id for the first time run.
  uint32_t nop_actor_id = 0;
  // Check the internal state and write down id for later use.
  bool sent_started, sent_cancel, interrupted;
  EXPECT_TRUE(action_queue.GetCurrentActionState(nullptr, &sent_started,
                                                 &sent_cancel, &interrupted,
                                                 &nop_actor_id, nullptr));
  EXPECT_TRUE(sent_started);
  EXPECT_FALSE(sent_cancel);
  EXPECT_FALSE(interrupted);
  ASSERT_NE(0u, nop_actor_id);

  // Add the next action which should ensure the first stopped.
  {
    UIntT uint;
    uint.val = 0;
    action_queue.EnqueueAction(nop_actor_factory.Make(uint));
  }

  // id for the second run.
  uint32_t nop_actor2_id = 0;
  // Check the internal state and write down id for later use.
  EXPECT_TRUE(action_queue.GetNextActionState(nullptr, &sent_started,
                                              &sent_cancel, &interrupted,
                                              &nop_actor2_id, nullptr));
  EXPECT_NE(nop_actor_id, nop_actor2_id);
  EXPECT_FALSE(sent_started);
  EXPECT_FALSE(sent_cancel);
  EXPECT_FALSE(interrupted);
  ASSERT_NE(0u, nop_actor2_id);

  action_queue.Tick();

  // Run the action so it can signal completion.
  event_loop_factory_.RunFor(chrono::seconds(1));

  action_queue.Tick();

  // Check the new action is the right one.
  uint32_t test_id = 0;
  EXPECT_TRUE(action_queue.GetCurrentActionState(
      nullptr, &sent_started, &sent_cancel, &interrupted, &test_id, nullptr));
  EXPECT_TRUE(sent_started);
  EXPECT_FALSE(sent_cancel);
  EXPECT_FALSE(interrupted);
  EXPECT_EQ(nop_actor2_id, test_id);

  // Make sure it is still going.
  EXPECT_TRUE(action_queue.Running());

  // Now let everything finish.
  event_loop_factory_.RunFor(chrono::seconds(1));
  action_queue.Tick();

  // Make sure it stopped.
  EXPECT_FALSE(action_queue.Running());
}

// Tests that we do get an index with our goal
TEST_F(ActionTest, ActionIndex) {
  TestActorIndex idx_actor(actor1_event_loop_.get());

  TestActorIndex::Factory test_actor_index_factory =
      TestActorIndex::MakeFactory(test_event_loop_.get());

  ActionQueue action_queue;
  // Tick an empty queue and make sure it was not running.  Also tick the
  // factory to allow it to send out the initial cancel message.
  event_loop_factory_.RunFor(chrono::seconds(1));
  action_queue.Tick();

  EXPECT_FALSE(action_queue.Running());

  // Enqueue action to post index.
  {
    UIntT uint;
    uint.val = 5;
    action_queue.EnqueueAction(test_actor_index_factory.Make(uint));
  }
  ::aos::Fetcher<actions::TestActionGoal> goal_fetcher_ =
      test_event_loop_->MakeFetcher<actions::TestActionGoal>(
          "/test_action");

  ASSERT_TRUE(goal_fetcher_.Fetch());
  EXPECT_EQ(5u, goal_fetcher_->params()->val());
  EXPECT_EQ(0u, idx_actor.index);

  action_queue.Tick();

  // Run the next action so it can accomplish signal completion.
  event_loop_factory_.RunFor(chrono::seconds(1));

  action_queue.Tick();
  EXPECT_EQ(5u, idx_actor.index);

  // Enqueue action to post index.
  {
    UIntT uint;
    uint.val = 3;
    action_queue.EnqueueAction(test_actor_index_factory.Make(uint));
  }
  ASSERT_TRUE(goal_fetcher_.Fetch());
  EXPECT_EQ(3u, goal_fetcher_->params()->val());

  // Run the next action so it can accomplish signal completion.
  event_loop_factory_.RunFor(chrono::seconds(1));

  action_queue.Tick();
  EXPECT_EQ(3u, idx_actor.index);
}

// Tests that an action with a structure params works.
TEST_F(ActionTest, StructParamType) {
  TestActor2Nop nop_actor(actor2_event_loop_.get());

  TestActor2Nop::Factory test_action_2_nop_factory =
      TestActor2Nop::MakeFactory(test_event_loop_.get());

  ActionQueue action_queue;
  // Tick an empty queue and make sure it was not running.
  action_queue.Tick();
  EXPECT_FALSE(action_queue.Running());

  actions::MyParamsT p;
  p.param1 = 5.0;
  p.param2 = 7;

  action_queue.EnqueueAction(test_action_2_nop_factory.Make(p));

  // We started an action and it should be running.
  EXPECT_TRUE(action_queue.Running());

  // Tick it and make sure it is still running.
  action_queue.Tick();
  EXPECT_TRUE(action_queue.Running());

  // Run the action so it can signal completion.
  // The actor takes no time, but running for a second is the best way to get it
  // to go.
  event_loop_factory_.RunFor(chrono::seconds(1));

  action_queue.Tick();

  // Make sure it stopped.
  EXPECT_FALSE(action_queue.Running());
}

}  // namespace testing
}  // namespace actions
}  // namespace common
}  // namespace aos
