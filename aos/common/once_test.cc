#include "aos/common/once.h"

#include <stdlib.h>
#include <limits.h>

#include "gtest/gtest.h"

namespace aos {
namespace testing {

class OnceTest : public ::testing::Test {
 public:
  static int *Function() {
    ++times_run_;
    value_ = 971 + times_run_;
    return &value_;
  }

 protected:
  void SetUp() {
    value_ = 0;
    times_run_ = 0;
  }

  static int value_;
  static int times_run_;
};
int OnceTest::value_, OnceTest::times_run_;

// Makes sure that it calls the function at the right time and that it correctly
// passes the result out.
TEST_F(OnceTest, Works) {
  static Once<int> once(Function);

  EXPECT_EQ(0, value_);
  EXPECT_EQ(0, times_run_);

  EXPECT_EQ(value_, *once.Get());
  EXPECT_NE(0, value_);
  EXPECT_NE(0, times_run_);
  // Make sure it's not passing it through an assignment by value or something
  // else weird.
  EXPECT_EQ(&value_, once.Get());
}

// Makes sure that having a Once at namespace scope works correctly.
namespace {

Once<int> global_once(OnceTest::Function);

}  // namespace

TEST_F(OnceTest, Global) {
  EXPECT_EQ(value_, *global_once.Get());
  EXPECT_NE(0, value_);
  EXPECT_NE(0, times_run_);
}

// Makes sure that an instance keeps returning the same value without running
// the function again.
TEST_F(OnceTest, MultipleGets) {
  static Once<int> once(Function);

  EXPECT_EQ(value_, *once.Get());
  EXPECT_EQ(1, times_run_);
  EXPECT_EQ(value_, *once.Get());
  EXPECT_EQ(1, times_run_);
  EXPECT_EQ(value_, *once.Get());
  EXPECT_EQ(1, times_run_);
}

// Tests to make sure that the right methods clear out the instance variables at
// the right times.
TEST_F(OnceTest, MemoryClearing) {
  Once<int> once(NULL);

  once.run_ = 1;
  once.done_ = true;
  // Run the constructor again to make sure it doesn't touch the variables set
  // above.
  new (&once)Once<int>(Function);

  // Should return a random, (potentially) uninitialized value.
  once.Get();
  EXPECT_EQ(0, times_run_);

  once.Reset();
  EXPECT_EQ(0, times_run_);
  EXPECT_EQ(value_, *once.Get());
  EXPECT_EQ(1, times_run_);
}

namespace {

int second_result = 0;
int *SecondFunction() {
  static int result = 254;
  second_result = ++result;
  return &second_result;
}

}  // namespace

// Makes sure that multiple instances don't interfere with each other.
TEST_F(OnceTest, MultipleInstances) {
  static Once<int> once1(Function);
  static Once<int> once2(SecondFunction);

  EXPECT_EQ(&value_, once1.Get());
  EXPECT_EQ(&second_result, once2.Get());
  EXPECT_EQ(&value_, once1.Get());
  EXPECT_EQ(&second_result, once2.Get());
}

// Tests calling Reset() to run the function a second time.
TEST_F(OnceTest, Recalculate) {
  Once<int> once(Function);
  once.Reset();

  EXPECT_EQ(value_, *once.Get());
  EXPECT_EQ(1, times_run_);

  value_ = 0;
  once.Reset();
  EXPECT_EQ(value_, *once.Get());
  EXPECT_EQ(2, times_run_);
}

}  // namespace testing
}  // namespace aos
