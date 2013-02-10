#include "TestAction.h"
#include "ResourceAction.h"
extern "C"{
#include <resource_internal.h>
}

#include <gtest/gtest.h>

#include "sharedmem_test_setup.h"

using namespace aos;

class AsyncActionTest : public ExecVeTestSetup{
	protected:
		virtual void SetUp(){
			AddProcess("../bin/TestAction");
			AddProcess("../bin/ResourceAction");
			//AddProcess("/home/brians/bin/wait_5s.sh");
			AddProcess("../bin/ResourceActionChild");

			ExecVeTestSetup::SetUp();
		}

		template<class H, class S, class T> bool HasResource(const uint16_t resource, H &handle){
			/*bool r = AOS_RESOURCE_STATE_GET_HAS_IT(resource, handle.GetInstance().resource_entity);
			EXPECT_EQ(r, handle.GetInstance().resources.count(resource)) << "the AsyncAction doesn't know whether it has resource " << resource << " or not";
			return r;*/

			/*const AsyncAction<S, T> &action = handle.GetInstance();
			async_action_start<S> *start = (async_action_start<S> *)aos_queue_read_message(action.start_queue, PEEK | NON_BLOCK);
			ASSERT_NE(NULL, start);
			bool r = AOS_RESOURCE_STATE_GET_HAS_IT(resource, start->parent);
			aos_queue_free_msg(action.start_queue, start);
			return r;*/

			AsyncAction<S, T> &action = (AsyncAction<S, T> &)handle.GetInstance();
			async_action_status<T> *status = (async_action_status<T> *)aos_queue_read_msg(action.status_queue, PEEK | NON_BLOCK);
			EXPECT_TRUE(status != NULL) << "if this failed, we're going to segfault next";
			bool r = AOS_RESOURCE_STATE_GET_HAS_IT(resource, status->resource_entity);
			aos_queue_free_msg(action.status_queue, status);
			return r;
		}

		// tests from the google doc (https://docs.google.com/document/d/1gzRrVcqL2X9VgNQUI5DrvLVVVziIH7c5ZerATVbiS7U/edit?hl=en_US) (referenced by the numbers in the drawing)
		// up here so they can be called with multiple inputs and checked for correct outputs
		// return = number of sub-action call it failed at (1 = top level one etc) (0 = succeeded)
		int GoogleDocTest1(uint8_t in_state){
			return ResourceAction.Execute(0x01, 0x01, 1, in_state).failed;
		}

		virtual void TearDown(){
			TestAction.Free();
			LeftDrive.Free();
			ResourceAction.Free();
			ResourceAction.Free();
			
			ExecVeTestSetup::TearDown();
		}
};

TEST_F(AsyncActionTest, StartStop){
	TestAction.Start(5, 3);
	TestAction.Stop();
	PercolatePause();
	EXPECT_TRUE(TestAction.IsDone());
}
TEST_F(AsyncActionTest, AlternateName){
	EXPECT_FALSE(LeftDrive.IsDone());
}

TEST_F(AsyncActionTest, Join){
	TestAction.Start(0.1, 3);
	EXPECT_FALSE(TestAction.IsDone());
	TestAction.Join();
	EXPECT_TRUE(TestAction.IsDone());
}
TEST_F(AsyncActionTest, JoinAgain){
	uint16_t instance = TestAction.Start(0.1, 3);
	TestAction.Join();
	EXPECT_TRUE(TestAction.IsDone());
	timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	TestAction.Join(instance);
	timespec ts2;
	clock_gettime(CLOCK_MONOTONIC, &ts2);
	long diff = ts2.tv_nsec - ts.tv_nsec;
	diff += (ts2.tv_sec - ts.tv_sec) * 1000000000;
	EXPECT_LT(diff, 50000);
}
TEST_F(AsyncActionTest, Execute){
	TestAction.Execute(0.06, 3);
	EXPECT_TRUE(TestAction.IsDone());
}

TEST_F(AsyncActionTest, Release){
	TestAction.Start(0.1, -4);
	while(!TestAction.IsDone()){
		if(TestAction.GetNextStatus())
			EXPECT_EQ(TestAction.GetLastStatus().loops > 2, !(HasResource<TestActionHandle, testing_status, testing_args>(test_resource1, TestAction)));
	}
}
TEST_F(AsyncActionTest, ReleaseBad){
	TestAction.Start(-0.01, 5);
}
TEST_F(AsyncActionTest, ImplicitRelease){
	TestAction.Execute(0.02, 4);
	EXPECT_FALSE((HasResource<TestActionHandle, testing_status, testing_args>(test_resource1, TestAction)));
}

//TODO test killing or not based on priority (and inherited priority...)

TEST_F(AsyncActionTest, GoogleDoc111){
	EXPECT_EQ(3, GoogleDocTest1(3));
}

