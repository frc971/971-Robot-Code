#include "aos/common/transaction.h"

#include <vector>

#include "gtest/gtest.h"

#include "aos/common/util/death_test_log_implementation.h"

namespace aos {
namespace transaction {
namespace testing {

class WorkStackTest : public ::testing::Test {
 public:
  // Contains an index which it adds to the created_works and invoked_works
  // vectors of its containing WorkStackTest.
  class TestWork {
   public:
    void Create(WorkStackTest *test, int i) {
      test->created_works()->push_back(i);
      i_ = i;
      test_ = test;
    }
    void DoWork() {
      test_->invoked_works()->push_back(i_);
    }

    int i() const { return i_; }

   private:
    int i_;
    WorkStackTest *test_;
  };

  ::std::vector<int> *created_works() { return &created_works_; }
  ::std::vector<int> *invoked_works() { return &invoked_works_; }
  WorkStack<TestWork, 20> *work_stack() { return &work_stack_; }

  // Creates a TestWork with index i and adds it to work_stack().
  void CreateWork(int i) {
    work_stack_.AddWork(this, i);
  }

 private:
  ::std::vector<int> created_works_, invoked_works_;
  WorkStack<TestWork, 20> work_stack_;
};

typedef WorkStackTest WorkStackDeathTest;

TEST_F(WorkStackTest, Basic) {
  EXPECT_FALSE(work_stack()->HasWork());
  EXPECT_EQ(0u, created_works()->size());
  EXPECT_EQ(0u, invoked_works()->size());

  CreateWork(971);
  EXPECT_TRUE(work_stack()->HasWork());
  EXPECT_EQ(1u, created_works()->size());
  EXPECT_EQ(0u, invoked_works()->size());
  EXPECT_EQ(971, created_works()->at(0));

  work_stack()->CompleteWork();
  EXPECT_FALSE(work_stack()->HasWork());
  EXPECT_EQ(1u, created_works()->size());
  EXPECT_EQ(1u, invoked_works()->size());
  EXPECT_EQ(971, invoked_works()->at(0));
}

TEST_F(WorkStackTest, DropWork) {
  CreateWork(971);
  CreateWork(254);
  EXPECT_EQ(2u, created_works()->size());

  work_stack()->DropWork();
  EXPECT_FALSE(work_stack()->HasWork());
  work_stack()->CompleteWork();
  EXPECT_EQ(0u, invoked_works()->size());
}

// Tests that the works get run in the correct order.
TEST_F(WorkStackTest, InvocationOrder) {
  CreateWork(971);
  CreateWork(254);
  CreateWork(1678);

  work_stack()->CompleteWork();
  EXPECT_EQ((::std::vector<int>{971, 254, 1678}), *created_works());
  EXPECT_EQ((::std::vector<int>{1678, 254, 971}), *invoked_works());
}

// Tests that it handles adding too many works intelligently.
TEST_F(WorkStackDeathTest, TooManyWorks) {
  logging::Init();
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        for (int i = 0; i < 1000; ++i) {
          CreateWork(i);
        }
      },
      ".*too many works.*");
}

}  // namespace testing
}  // namespace transaction
}  // namespace aos
