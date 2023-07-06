#include "glog/logging.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/constants/testdata/constants_data_generated.h"
#include "frc971/constants/testdata/constants_list_generated.h"

namespace frc971::constants {
namespace testing {

using aos::testing::ArtifactPath;

class ConstantSenderTest : public ::testing::Test {
 public:
  ConstantSenderTest()
      : config_(aos::configuration::ReadConfig(
            ArtifactPath("frc971/constants/testdata/aos_config.json"))),
        event_loop_factory_(&config_.message()),
        constants_sender_event_loop_(
            event_loop_factory_.MakeEventLoop("sender")) {}

  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::SimulatedEventLoopFactory event_loop_factory_;
  std::unique_ptr<aos::EventLoop> constants_sender_event_loop_;
};

// For team 971, compares the data that is recived from the program, to the data
// that is expected

TEST_F(ConstantSenderTest, HasData971) {
  aos::network::OverrideTeamNumber(971);

  std::unique_ptr<aos::EventLoop> test_event_loop =
      event_loop_factory_.MakeEventLoop("constants");
  ConstantSender<testdata::ConstantsData, testdata::ConstantsList> test971(
      constants_sender_event_loop_.get(),
      "frc971/constants/testdata/test_constants.json", "/constants");
  ConstantsFetcher<testdata::ConstantsData> fetcher(test_event_loop.get());
  EXPECT_EQ(fetcher.constants().max_roller_voltage(), 12);
  EXPECT_EQ(fetcher.constants().min_roller_voltage(), -12);
  // Ensure that the watcher in ConstantsFetcher never triggers.
  event_loop_factory_.RunFor(std::chrono::seconds(1));
}

// For team 9971, compares the data that is recived from the program, to the
// data that is expected.

TEST_F(ConstantSenderTest, HasData9971) {
  std::unique_ptr<aos::EventLoop> test_event_loop =
      event_loop_factory_.MakeEventLoop("constants");
  ConstantSender<testdata::ConstantsData, testdata::ConstantsList> test971(
      constants_sender_event_loop_.get(),
      "frc971/constants/testdata/test_constants.json", 9971, "/constants");
  ConstantsFetcher<testdata::ConstantsData> fetcher(test_event_loop.get());
  EXPECT_EQ(fetcher.constants().max_roller_voltage(), 6);
  EXPECT_EQ(fetcher.constants().min_roller_voltage(), -6);
  event_loop_factory_.RunFor(std::chrono::seconds(1));
}

// Tests that the ConstantsFetcher dies when there is no data available during
// construction.
TEST_F(ConstantSenderTest, NoDataOnStartup) {
  std::unique_ptr<aos::EventLoop> test_event_loop =
      event_loop_factory_.MakeEventLoop("constants");
  EXPECT_DEATH(ConstantsFetcher<testdata::ConstantsData>(test_event_loop.get()),
               "information must be available at startup");
}

// Tests that the ConstantsFetcher dies when there is a change to the constants
// data.
TEST_F(ConstantSenderTest, DieOnDataUpdate) {
  std::unique_ptr<aos::EventLoop> test_event_loop =
      event_loop_factory_.MakeEventLoop("constants");
  ConstantSender<testdata::ConstantsData, testdata::ConstantsList> test971(
      constants_sender_event_loop_.get(),
      "frc971/constants/testdata/test_constants.json", 9971, "/constants");
  ConstantsFetcher<testdata::ConstantsData> fetcher(test_event_loop.get());
  auto sender =
      constants_sender_event_loop_->MakeSender<testdata::ConstantsData>(
          "/constants");
  constants_sender_event_loop_->OnRun([&sender]() {
    auto builder = sender.MakeBuilder();
    builder.CheckOk(
        builder.Send(builder.MakeBuilder<testdata::ConstantsData>().Finish()));
  });
  EXPECT_DEATH(event_loop_factory_.RunFor(std::chrono::seconds(1)),
               "changes to constants");
}

// When given a team number that it not recognized we kill the program.

TEST_F(ConstantSenderTest, TeamNotFound) {
  EXPECT_DEATH(
      ({
        ConstantSender<testdata::ConstantsData, testdata::ConstantsList>
            test_no_team(constants_sender_event_loop_.get(),
                         "frc971/constants/testdata/test_constants.json", 254,
                         "/constants");
        event_loop_factory_.RunFor(std::chrono::seconds(1));
      }),
      "There was no match for 254");
}

// If the json file has syntax errors it will die.

TEST_F(ConstantSenderTest, SyntaxErrorDeath) {
  EXPECT_DEATH(
      ({
        ConstantSender<testdata::ConstantsData, testdata::ConstantsList>
            test_syntax(constants_sender_event_loop_.get(),
                        "frc971/constants/testdata/syntax_error.json", 971,
                        "/constants");
        event_loop_factory_.RunFor(std::chrono::seconds(1));
      }),
      "Invalid field name");
}

}  // namespace testing
}  // namespace frc971::constants
