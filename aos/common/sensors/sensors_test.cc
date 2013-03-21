#include "aos/common/sensors/sensors.h"

#include <stdint.h>

#include "gtest/gtest.h"

#include "aos/common/queue_testutils.h"

namespace aos {
namespace sensors {
namespace testing {

struct TestValues {
  int32_t data1, data2;
};

TEST(SensorDataTest, Checksum) {
  ::aos::common::testing::EnableTestLogging();

  SensorData<TestValues> data;
  data.values.data1 = 0;
  data.values.data2 = 5;
  data.FillinChecksum();
  EXPECT_TRUE(data.CheckChecksum());
  data.values.data1 = 1;
  EXPECT_FALSE(data.CheckChecksum());
  data.values.data1 = 0xFFFFFFFF;
  EXPECT_FALSE(data.CheckChecksum());
  data.values.data1 = 0;
  EXPECT_TRUE(data.CheckChecksum());
  data.values.data1 = 5;
  data.values.data2 = 0;
  EXPECT_FALSE(data.CheckChecksum());
}

}  // namespace testing
}  // namespace sensors
}  // namespace aos
