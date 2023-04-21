#include "aos/events/logging/log_reader_utils.h"

#include "aos/events/logging/file_operations.h"
#include "aos/events/logging/multinode_logger_test_lib.h"
#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/testing/tmpdir.h"

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

// Test to run log reader with replay channels via simulated event loop
TEST_P(MultinodeLoggerOneConfigTest, SingleNodeLogReplay) {
  time_converter_.StartEqual();
  std::vector<std::string> actual_filenames;
  const std::string kLogfile1_1 =
      aos::testing::TestTmpDir() + "/multi_logfile1/";
  util::UnlinkRecursive(kLogfile1_1);

  {
    LoggerState pi1_logger = MakeLoggerState(
        pi1_, &event_loop_factory_, SupportedCompressionAlgorithms()[0]);
    pi2_->DisableStatistics();
    pi2_->Disconnect(pi1_->node());
    pi1_->Disconnect(pi2_->node());
    pi1_logger.StartLogger(kLogfile1_1);
    event_loop_factory_.RunFor(chrono::milliseconds(20000));
    pi1_logger.AppendAllFilenames(&actual_filenames);
  }

  ReplayChannels replay_channels{{"/test", "aos.examples.Ping"}};
  LogReader reader(logger::SortParts(actual_filenames), &config_.message(),
                   &replay_channels);
  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  int ping_count = 0;
  int pong_count = 0;

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");

  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  pi1_event_loop->MakeWatcher(
      "/test", [&ping_count](const examples::Ping &) { ++ping_count; });
  pi1_event_loop->MakeWatcher(
      "/test", [&pong_count](const examples::Pong &) { ++pong_count; });

  int sent_messages = 1999;
  reader.event_loop_factory()->Run();
  EXPECT_EQ(ping_count, sent_messages);
  EXPECT_EQ(pong_count, 0);
  reader.Deregister();
}

// Verify that it is OK to list single file.
TEST(FileOperationTest, SingleFile) {
  std::string log_file = aos::testing::TestTmpDir() + "/test.bfbs";
  util::WriteStringToFileOrDie(log_file, "test");
  internal::LocalFileOperations file_op(log_file);
  EXPECT_TRUE(file_op.Exists());
  std::vector<std::string> logs;
  file_op.FindLogs(&logs);
  ASSERT_EQ(logs.size(), 1);
  EXPECT_EQ(logs.front(), log_file);
}

// Verify that it is OK to list folder with log file.
TEST(FileOperationTest, ListDirectory) {
  std::string log_folder = aos::testing::TestTmpDir() + "/log_folder/";
  std::filesystem::create_directories(log_folder);
  std::string log_file = log_folder + "test.bfbs";
  util::WriteStringToFileOrDie(log_file, "test");
  internal::LocalFileOperations file_op(log_folder);
  EXPECT_TRUE(file_op.Exists());
  std::vector<std::string> logs;
  file_op.FindLogs(&logs);
  ASSERT_EQ(logs.size(), 1);
  EXPECT_EQ(logs.front(), log_file);
}

}  // namespace aos::logger::testing
