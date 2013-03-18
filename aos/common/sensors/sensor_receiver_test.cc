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
  void DoReceiveData() {
    last_received_count_ = ++data()->count;
    data()->values.count = last_received_count_;
    Time::IncrementMockTime(kSensorSendFrequency);
    data()->HostToNetwork();
  }

  int resets() { return resets_; }
  int unpacks() { return unpacks_; }
  using SensorReceiver<TestValues>::data;

  void UnpackFrom(TestValues *data) {
    // Make sure that it didn't lost one that we gave it.
    EXPECT_EQ(last_received_count_, data->count);
    ++unpacks_;
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

// TODO(brians) finish writing tests

}  // namespace testing
}  // namespace sensors
}  // namespace aos
