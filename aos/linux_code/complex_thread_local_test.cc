#include "aos/linux_code/complex_thread_local.h"

#include <atomic>

#include "gtest/gtest.h"

#include "aos/common/util/thread.h"

namespace aos {
namespace testing {

class ComplexThreadLocalTest : public ::testing::Test {
 protected:
  struct TraceableObject {
    TraceableObject(int data = 0) : data(data) { ++constructions; }
    ~TraceableObject() { ++destructions; }

    static ::std::atomic<int> constructions, destructions;

    int data;
  };
  ComplexThreadLocal<TraceableObject> local;

 private:
  void SetUp() override {
    local.Clear();
    EXPECT_EQ(TraceableObject::constructions, TraceableObject::destructions)
        << "There should be no way to create and destroy different numbers.";
    TraceableObject::constructions = TraceableObject::destructions = 0;
  }
};
::std::atomic<int> ComplexThreadLocalTest::TraceableObject::constructions;
::std::atomic<int> ComplexThreadLocalTest::TraceableObject::destructions;

TEST_F(ComplexThreadLocalTest, Basic) {
  EXPECT_EQ(0, TraceableObject::constructions);
  EXPECT_EQ(0, TraceableObject::destructions);
  EXPECT_FALSE(local.created());
  EXPECT_EQ(nullptr, local.get());

  local.Create(971);
  EXPECT_EQ(1, TraceableObject::constructions);
  EXPECT_EQ(0, TraceableObject::destructions);
  EXPECT_TRUE(local.created());
  EXPECT_EQ(971, local->data);

  local.Create(254);
  EXPECT_EQ(1, TraceableObject::constructions);
  EXPECT_EQ(0, TraceableObject::destructions);
  EXPECT_TRUE(local.created());
  EXPECT_EQ(971, local->data);

  local.Clear();
  EXPECT_EQ(1, TraceableObject::constructions);
  EXPECT_EQ(1, TraceableObject::destructions);
  EXPECT_FALSE(local.created());
  EXPECT_EQ(nullptr, local.get());

  local.Create(973);
  EXPECT_EQ(2, TraceableObject::constructions);
  EXPECT_EQ(1, TraceableObject::destructions);
  EXPECT_TRUE(local.created());
  EXPECT_EQ(973, local->data);
}

TEST_F(ComplexThreadLocalTest, AnotherThread) {
  EXPECT_FALSE(local.created());
  util::FunctionThread::RunInOtherThread([this]() {
    EXPECT_FALSE(local.created());
    local.Create(971);
    EXPECT_TRUE(local.created());
    EXPECT_EQ(971, local->data);
    EXPECT_EQ(1, TraceableObject::constructions);
    EXPECT_EQ(0, TraceableObject::destructions);
  });
  EXPECT_EQ(1, TraceableObject::constructions);
  EXPECT_EQ(1, TraceableObject::destructions);
  EXPECT_FALSE(local.created());
}

TEST_F(ComplexThreadLocalTest, TwoThreads) {
  util::FunctionThread thread([this](util::FunctionThread *) {
    local.Create(971);
    EXPECT_EQ(971, local->data);
    EXPECT_EQ(0, TraceableObject::destructions);
  });
  thread.Start();
  local.Create(973);
  EXPECT_EQ(973, local->data);
  thread.Join();
  EXPECT_TRUE(local.created());
  EXPECT_EQ(2, TraceableObject::constructions);
  EXPECT_EQ(1, TraceableObject::destructions);
  local.Clear();
  EXPECT_EQ(2, TraceableObject::constructions);
  EXPECT_EQ(2, TraceableObject::destructions);
  EXPECT_FALSE(local.created());
}

}  // namespace testing
}  // namespace aos
