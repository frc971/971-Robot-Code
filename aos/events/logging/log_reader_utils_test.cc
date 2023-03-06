#include "aos/events/logging/log_reader_utils.h"

#include "aos/events/logging/multinode_logger_test_lib.h"

namespace aos::logger::testing {

namespace chrono = std::chrono;
// Created this test fixture because the test case checks for channel names
// which are different in different configs
using MultinodeLoggerOneConfigTest = MultinodeLoggerTest;

INSTANTIATE_TEST_SUITE_P(
    All, MultinodeLoggerOneConfigTest,
    ::testing::Combine(::testing::Values(ConfigParams{
                           "multinode_pingpong_combined_config.json", true,
                           kCombinedConfigSha1(), kCombinedConfigSha1()}),
                       ::testing::ValuesIn(SupportedCompressionAlgorithms())));

// This test is to check if we are able to get the right channels from a log
// given nodes and applications using the function ChannelsInLog
TEST_P(MultinodeLoggerOneConfigTest, ChannelsInLogTest) {
  // Run the logger
  time_converter_.StartEqual();
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  auto sorted_parts = SortParts(logfiles_);
  // Read all the sorted log files
  LogReader reader(sorted_parts);

  std::vector<const Node *> active_nodes;
  std::vector<std::string> applications;
  // Get the active node
  active_nodes.emplace_back(
      configuration::GetNode(reader.configuration(), "pi1"));

  // Get the application for which you want to check channels
  applications.push_back("ping");
  aos::logger::ChannelsInLogResult channels =
      aos::logger::ChannelsInLog(sorted_parts, active_nodes, applications);

  // Check for the right sender channels
  std::vector<std::string> expected_senders;
  expected_senders.push_back("/pi1/aos aos.logging.LogMessageFbs");
  expected_senders.push_back("/pi1/aos aos.timing.Report");
  expected_senders.push_back("/test aos.examples.Ping");

  std::vector<std::string> check_senders;
  for (const auto &sender : channels.senders.value()) {
    check_senders.push_back(sender.name + " " + sender.type);
  }
  ASSERT_THAT(check_senders,
              ::testing::UnorderedElementsAreArray(expected_senders));
  ASSERT_EQ(channels.senders.value().size(), 3);

  // Check for the right watcher channels
  std::vector<std::string> expected_watchers;
  expected_watchers.push_back("/test aos.examples.Pong");
  std::vector<std::string> check_watchers;
  for (const auto &watcher : channels.watchers.value()) {
    check_watchers.push_back(watcher.name + " " + watcher.type);
  }
  ASSERT_THAT(check_watchers,
              ::testing::UnorderedElementsAreArray(expected_watchers));
  ASSERT_EQ(channels.watchers.value().size(), 1);

  // There no fetcher channels, check for none
  ASSERT_EQ(channels.fetchers.value().size(), 0);
}
}  // namespace aos::logger::testing
