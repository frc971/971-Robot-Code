#include "aos/events/timing_statistics.h"

#include "aos/flatbuffers.h"
#include "gtest/gtest.h"

namespace aos {
namespace internal {
namespace testing {

TEST(TimingStatistic, StatisticsTest) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(timing::CreateStatistic(fbb));

  FlatbufferDetachedBuffer<timing::Statistic> statistic(fbb.Release());

  TimingStatistic ts;
  ts.set_statistic(statistic.mutable_message());

  // Make sure we can add 2 numbers and get the expected result.
  ts.Add(5.0);

  EXPECT_EQ(statistic.message().average(), 5.0);
  EXPECT_EQ(statistic.message().min(), 5.0);
  EXPECT_EQ(statistic.message().max(), 5.0);
  EXPECT_EQ(statistic.message().standard_deviation(), 0.0);

  ts.Add(5.0);

  EXPECT_EQ(statistic.message().average(), 5.0);
  EXPECT_EQ(statistic.message().min(), 5.0);
  EXPECT_EQ(statistic.message().max(), 5.0);
  EXPECT_EQ(statistic.message().standard_deviation(), 0.0);

  // Make sure reset works.
  ts.Reset();

  // And all the results are nan.
  EXPECT_TRUE(std::isnan(statistic.message().average()));
  EXPECT_TRUE(std::isnan(statistic.message().min()));
  EXPECT_TRUE(std::isnan(statistic.message().max()));
  EXPECT_TRUE(std::isnan(statistic.message().standard_deviation()));

  ts.Add(7.0);

  EXPECT_EQ(statistic.message().average(), 7.0);
  EXPECT_EQ(statistic.message().min(), 7.0);
  EXPECT_EQ(statistic.message().max(), 7.0);
  EXPECT_EQ(statistic.message().standard_deviation(), 0.0);

  ts.Reset();

  // Now add a predetermined set of data and make sure we get a result which
  // agrees with online calculators.
  ts.Add(10);
  ts.Add(12);
  ts.Add(23);
  ts.Add(23);
  ts.Add(16);
  ts.Add(23);
  ts.Add(21);
  ts.Add(16);

  EXPECT_EQ(statistic.message().average(), 18.0);
  EXPECT_EQ(statistic.message().min(), 10.0);
  EXPECT_EQ(statistic.message().max(), 23.0);
  EXPECT_NEAR(statistic.message().standard_deviation(), 5.2372293656638, 1e-6);
}

}  // namespace testing
}  // namespace internal
}  // namespace aos
