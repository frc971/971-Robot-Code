#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/constants/testdata/constants_data_generated.h"
#include "frc971/constants/testdata/constants_list_generated.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

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
  test_event_loop->MakeWatcher("/constants",
                               [](const testdata::ConstantsData &data) {
                                 EXPECT_EQ(data.max_roller_voltage(), 12);
                                 EXPECT_EQ(data.min_roller_voltage(), -12);
                               });
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
  test_event_loop->MakeWatcher("/constants",
                               [](const testdata::ConstantsData &data) {
                                 EXPECT_EQ(data.max_roller_voltage(), 6);
                                 EXPECT_EQ(data.min_roller_voltage(), -6);
                               });
  event_loop_factory_.RunFor(std::chrono::seconds(1));
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
                        "frc971/constants/testdata/syntaxerror.json", 971,
                        "/constants");
        event_loop_factory_.RunFor(std::chrono::seconds(1));
      }),
      "Error on line 0");
}

}  // namespace testing
}  // namespace frc971::constants
