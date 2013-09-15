#include "aos/common/sensors/sensor_receiver.h"

#include "gtest/gtest.h"

#include "aos/common/sensors/sensors.h"
#include "aos/common/time.h"
#include "aos/common/queue_testutils.h"

using ::aos::time::Time;

namespace aos {
namespace sensors {
namespace testing {

struct TestValues {
  int count;
  int more_data;
};
class TestSensorReceiver : public SensorReceiver<TestValues>,
    public SensorUnpackerInterface<TestValues> {
 public:
  TestSensorReceiver()
      : SensorReceiver<TestValues>(this),
        resets_(0),
        unpacks_(0) {
    data()->count = 0;
  }

  void Reset() {
    LOG(DEBUG, "reset for the %dth time\n", ++resets_);
  }
  bool DoReceiveData() {
    last_received_count_ = ++data()->count;
    data()->values.count = last_received_count_;
    Time::IncrementMockTime(kSensorSendFrequency);
    data()->FillinChecksum();
    data()->HostToNetwork();
    return false;
  }

  int resets() { return resets_; }
  int unpacks() { return unpacks_; }
  using SensorReceiver<TestValues>::data;

  void ResetFakeData() {
    data()->count = 0;
  }

  void UnpackFrom(TestValues *data) {
    // Make sure that it didn't lose one that we gave it.
    EXPECT_EQ(last_received_count_, data->count);
    ++unpacks_;
    LOG(DEBUG, "%dth unpack\n", unpacks_);
  }
 
 private:
  int resets_;
  int unpacks_;
  int last_received_count_;
};

class SensorReceiverTest : public ::testing::Test {
 protected:
  SensorReceiverTest() {
    ::aos::common::testing::EnableTestLogging();
    Time::EnableMockTime(Time(971, 254));
  }

  TestSensorReceiver &receiver() { return receiver_; }

 private:
  TestSensorReceiver receiver_;
};

TEST_F(SensorReceiverTest, Simple) {
  static const int kIterations = 53;
  for (int i = 0; i < kIterations; ++i) {
    receiver().RunIteration();
  }
  EXPECT_EQ(1, receiver().resets());
  // expected value is kIterations/kSendsPerCycle (rounded up) (the number of
  // times that it should get a good one) - 1 (to compensate for the iteration
  // when it synced itself up)
  EXPECT_EQ((kIterations + kSendsPerCycle - 1) / kSendsPerCycle - 1,
            receiver().unpacks());
}

TEST_F(SensorReceiverTest, CRIOReboot) {
  for (int i = 0; i < 50; ++i) {
    receiver().RunIteration();
    if (i == 27) {
      receiver().ResetFakeData();
      time::Time::IncrementMockTime(time::Time::InSeconds(20));
    }
  }
  EXPECT_EQ(2, receiver().resets());
  EXPECT_GE(receiver().unpacks(), 4);
}

TEST_F(SensorReceiverTest, CRIOSkew) {
  for (int i = 0; i < 505; ++i) {
    receiver().RunIteration();
    time::Time::IncrementMockTime(time::Time(0, 4000));
  }
  // TODO(brians) verify here that it actually corrects (happens twice with
  // current constants)
  EXPECT_EQ(1, receiver().resets());
  EXPECT_EQ(50, receiver().unpacks());
}

TEST_F(SensorReceiverTest, BadStartup1) {
  time::Time::SetMockTime(NextLoopTime() - Time(0, 100));
  for (int i = 0; i < 55; ++i) {
    receiver().RunIteration();
  }
  EXPECT_EQ(1, receiver().resets());
  EXPECT_EQ(5, receiver().unpacks());
}

TEST_F(SensorReceiverTest, BadStartup2) {
  time::Time::SetMockTime(NextLoopTime() -
                          SensorReceiver<TestValues>::kJitterDelay -
                          time::Time(0, 1));
  for (int i = 0; i < 55; ++i) {
    receiver().RunIteration();
  }
  EXPECT_EQ(2, receiver().resets());
  EXPECT_EQ(5, receiver().unpacks());
}

TEST_F(SensorReceiverTest, BadStartup3) {
  time::Time::SetMockTime(NextLoopTime() -
                          time::Time::InSeconds(0.002) +
                          kLoopFrequency / 20);
  for (int i = 0; i < 55; ++i) {
    receiver().RunIteration();
  }
  EXPECT_EQ(1, receiver().resets());
  EXPECT_EQ(5, receiver().unpacks());
}

// I think that it somehow got this way once and never recovered.
// It should never get this way, but if it does, it should recover.
TEST_F(SensorReceiverTest, StartTimeAndCountMismatch) {
  for (int i = 0; i < 1005; ++i) {
    receiver().RunIteration();
    if (i == 3) {
      receiver().start_count_ += 10;
    }
  }
  EXPECT_EQ(2, receiver().resets());
  EXPECT_GT(receiver().unpacks(), 30);
}

// TODO(brians) finish writing tests and commenting them and the code

}  // namespace testing
}  // namespace sensors
}  // namespace aos
