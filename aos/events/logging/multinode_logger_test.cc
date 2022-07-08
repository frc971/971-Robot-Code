#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/multinode_logger_test_lib.h"
#include "aos/events/message_counter.h"
#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/timestamp_generated.h"
#include "aos/testing/tmpdir.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace aos {
namespace logger {
namespace testing {

namespace chrono = std::chrono;
using aos::message_bridge::RemoteMessage;
using aos::testing::ArtifactPath;
using aos::testing::MessageCounter;

INSTANTIATE_TEST_SUITE_P(
    All, MultinodeLoggerTest,
    ::testing::Combine(
        ::testing::Values(
            ConfigParams{"multinode_pingpong_combined_config.json", true,
                         kCombinedConfigSha1(), kCombinedConfigSha1()},
            ConfigParams{"multinode_pingpong_split_config.json", false,
                         kSplitConfigSha1(), kReloggedSplitConfigSha1()}),
        ::testing::ValuesIn(SupportedCompressionAlgorithms())));

INSTANTIATE_TEST_SUITE_P(
    All, MultinodeLoggerDeathTest,
    ::testing::Combine(
        ::testing::Values(
            ConfigParams{"multinode_pingpong_combined_config.json", true,
                         kCombinedConfigSha1(), kCombinedConfigSha1()},
            ConfigParams{"multinode_pingpong_split_config.json", false,
                         kSplitConfigSha1(), kReloggedSplitConfigSha1()}),
        ::testing::ValuesIn(SupportedCompressionAlgorithms())));

// Tests that we can write and read simple multi-node log files.
TEST_P(MultinodeLoggerTest, SimpleMultiNode) {
  std::vector<std::string> actual_filenames;
  time_converter_.StartEqual();

  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
    pi1_logger.AppendAllFilenames(&actual_filenames);
    pi2_logger.AppendAllFilenames(&actual_filenames);
  }

  ASSERT_THAT(actual_filenames,
              ::testing::UnorderedElementsAreArray(logfiles_));

  {
    std::set<std::string> logfile_uuids;
    std::set<std::string> parts_uuids;
    // Confirm that we have the expected number of UUIDs for both the logfile
    // UUIDs and parts UUIDs.
    std::vector<SizePrefixedFlatbufferVector<LogFileHeader>> log_header;
    for (std::string_view f : logfiles_) {
      log_header.emplace_back(ReadHeader(f).value());
      if (!log_header.back().message().has_configuration()) {
        logfile_uuids.insert(
            log_header.back().message().log_event_uuid()->str());
        parts_uuids.insert(log_header.back().message().parts_uuid()->str());
      }
    }

    EXPECT_EQ(logfile_uuids.size(), 2u);
    if (shared()) {
      EXPECT_EQ(parts_uuids.size(), 7u);
    } else {
      EXPECT_EQ(parts_uuids.size(), 8u);
    }

    // And confirm everything is on the correct node.
    EXPECT_EQ(log_header[2].message().node()->name()->string_view(), "pi1");
    EXPECT_EQ(log_header[3].message().node()->name()->string_view(), "pi1");
    EXPECT_EQ(log_header[4].message().node()->name()->string_view(), "pi1");

    EXPECT_EQ(log_header[5].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[6].message().node()->name()->string_view(), "pi2");

    EXPECT_EQ(log_header[7].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[8].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[9].message().node()->name()->string_view(), "pi2");

    EXPECT_EQ(log_header[10].message().node()->name()->string_view(), "pi1");
    EXPECT_EQ(log_header[11].message().node()->name()->string_view(), "pi1");

    EXPECT_EQ(log_header[12].message().node()->name()->string_view(), "pi2");
    EXPECT_EQ(log_header[13].message().node()->name()->string_view(), "pi2");

    if (shared()) {
      EXPECT_EQ(log_header[14].message().node()->name()->string_view(), "pi2");
      EXPECT_EQ(log_header[15].message().node()->name()->string_view(), "pi2");
      EXPECT_EQ(log_header[16].message().node()->name()->string_view(), "pi2");

      EXPECT_EQ(log_header[17].message().node()->name()->string_view(), "pi1");
      EXPECT_EQ(log_header[18].message().node()->name()->string_view(), "pi1");
    } else {
      EXPECT_EQ(log_header[14].message().node()->name()->string_view(), "pi2");
      EXPECT_EQ(log_header[15].message().node()->name()->string_view(), "pi2");

      EXPECT_EQ(log_header[16].message().node()->name()->string_view(), "pi1");
      EXPECT_EQ(log_header[17].message().node()->name()->string_view(), "pi1");

      EXPECT_EQ(log_header[18].message().node()->name()->string_view(), "pi2");
      EXPECT_EQ(log_header[19].message().node()->name()->string_view(), "pi2");
    }

    // And the parts index matches.
    EXPECT_EQ(log_header[2].message().parts_index(), 0);
    EXPECT_EQ(log_header[3].message().parts_index(), 1);
    EXPECT_EQ(log_header[4].message().parts_index(), 2);

    EXPECT_EQ(log_header[5].message().parts_index(), 0);
    EXPECT_EQ(log_header[6].message().parts_index(), 1);

    EXPECT_EQ(log_header[7].message().parts_index(), 0);
    EXPECT_EQ(log_header[8].message().parts_index(), 1);
    EXPECT_EQ(log_header[9].message().parts_index(), 2);

    EXPECT_EQ(log_header[10].message().parts_index(), 0);
    EXPECT_EQ(log_header[11].message().parts_index(), 1);

    EXPECT_EQ(log_header[12].message().parts_index(), 0);
    EXPECT_EQ(log_header[13].message().parts_index(), 1);

    if (shared()) {
      EXPECT_EQ(log_header[14].message().parts_index(), 0);
      EXPECT_EQ(log_header[15].message().parts_index(), 1);
      EXPECT_EQ(log_header[16].message().parts_index(), 2);

      EXPECT_EQ(log_header[17].message().parts_index(), 0);
      EXPECT_EQ(log_header[18].message().parts_index(), 1);
    } else {
      EXPECT_EQ(log_header[14].message().parts_index(), 0);
      EXPECT_EQ(log_header[15].message().parts_index(), 1);

      EXPECT_EQ(log_header[16].message().parts_index(), 0);
      EXPECT_EQ(log_header[17].message().parts_index(), 1);

      EXPECT_EQ(log_header[18].message().parts_index(), 0);
      EXPECT_EQ(log_header[19].message().parts_index(), 1);
    }
  }

  const std::vector<LogFile> sorted_log_files = SortParts(logfiles_);
  {
    using ::testing::UnorderedElementsAre;
    std::shared_ptr<const aos::Configuration> config =
        sorted_log_files[0].config;

    // Timing reports, pings
    EXPECT_THAT(CountChannelsData(config, logfiles_[2]),
                UnorderedElementsAre(
                    std::make_tuple("/pi1/aos",
                                    "aos.message_bridge.ServerStatistics", 1),
                    std::make_tuple("/test", "aos.examples.Ping", 1),
                    std::make_tuple("/pi1/aos", "aos.examples.Ping", 1)))
        << " : " << logfiles_[2];
    {
      std::vector<std::tuple<std::string, std::string, int>> channel_counts = {
          std::make_tuple("/pi1/aos", "aos.examples.Ping", 10),
          std::make_tuple("/pi1/aos", "aos.message_bridge.Timestamp", 1),
          std::make_tuple("/pi1/aos", "aos.message_bridge.ClientStatistics",
                          1)};
      if (!std::get<0>(GetParam()).shared) {
        channel_counts.push_back(
            std::make_tuple("/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                            "aos-message_bridge-Timestamp",
                            "aos.message_bridge.RemoteMessage", 1));
      }
      EXPECT_THAT(CountChannelsData(config, logfiles_[3]),
                  ::testing::UnorderedElementsAreArray(channel_counts))
          << " : " << logfiles_[3];
    }
    {
      std::vector<std::tuple<std::string, std::string, int>> channel_counts = {
          std::make_tuple("/pi1/aos", "aos.examples.Ping", 1990),
          std::make_tuple("/pi1/aos", "aos.message_bridge.Timestamp", 199),
          std::make_tuple("/pi1/aos", "aos.message_bridge.ServerStatistics",
                          20),
          std::make_tuple("/pi1/aos", "aos.message_bridge.ClientStatistics",
                          199),
          std::make_tuple("/pi1/aos", "aos.timing.Report", 60),
          std::make_tuple("/test", "aos.examples.Ping", 2000)};
      if (!std::get<0>(GetParam()).shared) {
        channel_counts.push_back(
            std::make_tuple("/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                            "aos-message_bridge-Timestamp",
                            "aos.message_bridge.RemoteMessage", 199));
      }
      EXPECT_THAT(CountChannelsData(config, logfiles_[4]),
                  ::testing::UnorderedElementsAreArray(channel_counts))
          << " : " << logfiles_[4];
    }
    // Timestamps for pong
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[2]),
                UnorderedElementsAre())
        << " : " << logfiles_[2];
    EXPECT_THAT(
        CountChannelsTimestamp(config, logfiles_[3]),
        UnorderedElementsAre(std::make_tuple("/test", "aos.examples.Pong", 1)))
        << " : " << logfiles_[3];
    EXPECT_THAT(
        CountChannelsTimestamp(config, logfiles_[4]),
        UnorderedElementsAre(
            std::make_tuple("/test", "aos.examples.Pong", 2000),
            std::make_tuple("/pi2/aos", "aos.message_bridge.Timestamp", 200)))
        << " : " << logfiles_[4];

    // Pong data.
    EXPECT_THAT(
        CountChannelsData(config, logfiles_[5]),
        UnorderedElementsAre(std::make_tuple("/test", "aos.examples.Pong", 91)))
        << " : " << logfiles_[5];
    EXPECT_THAT(CountChannelsData(config, logfiles_[6]),
                UnorderedElementsAre(
                    std::make_tuple("/test", "aos.examples.Pong", 1910)))
        << " : " << logfiles_[6];

    // No timestamps
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[5]),
                UnorderedElementsAre())
        << " : " << logfiles_[5];
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[6]),
                UnorderedElementsAre())
        << " : " << logfiles_[6];

    // Timing reports and pongs.
    EXPECT_THAT(CountChannelsData(config, logfiles_[7]),
                UnorderedElementsAre(
                    std::make_tuple("/pi2/aos", "aos.examples.Ping", 1),
                    std::make_tuple("/pi2/aos",
                                    "aos.message_bridge.ServerStatistics", 1)))
        << " : " << logfiles_[7];
    EXPECT_THAT(
        CountChannelsData(config, logfiles_[8]),
        UnorderedElementsAre(std::make_tuple("/test", "aos.examples.Pong", 1)))
        << " : " << logfiles_[8];
    EXPECT_THAT(
        CountChannelsData(config, logfiles_[9]),
        UnorderedElementsAre(
            std::make_tuple("/pi2/aos", "aos.examples.Ping", 2000),
            std::make_tuple("/pi2/aos", "aos.message_bridge.Timestamp", 200),
            std::make_tuple("/pi2/aos", "aos.message_bridge.ServerStatistics",
                            20),
            std::make_tuple("/pi2/aos", "aos.message_bridge.ClientStatistics",
                            200),
            std::make_tuple("/pi2/aos", "aos.timing.Report", 60),
            std::make_tuple("/test", "aos.examples.Pong", 2000)))
        << " : " << logfiles_[9];
    // And ping timestamps.
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[7]),
                UnorderedElementsAre())
        << " : " << logfiles_[7];
    EXPECT_THAT(
        CountChannelsTimestamp(config, logfiles_[8]),
        UnorderedElementsAre(std::make_tuple("/test", "aos.examples.Ping", 1)))
        << " : " << logfiles_[8];
    EXPECT_THAT(
        CountChannelsTimestamp(config, logfiles_[9]),
        UnorderedElementsAre(
            std::make_tuple("/test", "aos.examples.Ping", 2000),
            std::make_tuple("/pi1/aos", "aos.message_bridge.Timestamp", 200)))
        << " : " << logfiles_[9];

    // And then test that the remotely logged timestamp data files only have
    // timestamps in them.
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[10]),
                UnorderedElementsAre())
        << " : " << logfiles_[10];
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[11]),
                UnorderedElementsAre())
        << " : " << logfiles_[11];
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[12]),
                UnorderedElementsAre())
        << " : " << logfiles_[12];
    EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[13]),
                UnorderedElementsAre())
        << " : " << logfiles_[13];

    EXPECT_THAT(CountChannelsData(config, logfiles_[10]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi1/aos", "aos.message_bridge.Timestamp", 9)))
        << " : " << logfiles_[10];
    EXPECT_THAT(CountChannelsData(config, logfiles_[11]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi1/aos", "aos.message_bridge.Timestamp", 191)))
        << " : " << logfiles_[11];

    EXPECT_THAT(CountChannelsData(config, logfiles_[12]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi2/aos", "aos.message_bridge.Timestamp", 9)))
        << " : " << logfiles_[12];
    EXPECT_THAT(CountChannelsData(config, logfiles_[13]),
                UnorderedElementsAre(std::make_tuple(
                    "/pi2/aos", "aos.message_bridge.Timestamp", 191)))
        << " : " << logfiles_[13];

    // Timestamps from pi2 on pi1, and the other way.
    if (shared()) {
      EXPECT_THAT(CountChannelsData(config, logfiles_[14]),
                  UnorderedElementsAre())
          << " : " << logfiles_[14];
      EXPECT_THAT(CountChannelsData(config, logfiles_[15]),
                  UnorderedElementsAre())
          << " : " << logfiles_[15];
      EXPECT_THAT(CountChannelsData(config, logfiles_[16]),
                  UnorderedElementsAre())
          << " : " << logfiles_[16];
      EXPECT_THAT(CountChannelsData(config, logfiles_[17]),
                  UnorderedElementsAre())
          << " : " << logfiles_[17];
      EXPECT_THAT(CountChannelsData(config, logfiles_[18]),
                  UnorderedElementsAre())
          << " : " << logfiles_[18];

      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[14]),
                  UnorderedElementsAre(
                      std::make_tuple("/test", "aos.examples.Ping", 1)))
          << " : " << logfiles_[14];
      EXPECT_THAT(
          CountChannelsTimestamp(config, logfiles_[15]),
          UnorderedElementsAre(
              std::make_tuple("/pi1/aos", "aos.message_bridge.Timestamp", 9),
              std::make_tuple("/test", "aos.examples.Ping", 90)))
          << " : " << logfiles_[15];
      EXPECT_THAT(
          CountChannelsTimestamp(config, logfiles_[16]),
          UnorderedElementsAre(
              std::make_tuple("/pi1/aos", "aos.message_bridge.Timestamp", 191),
              std::make_tuple("/test", "aos.examples.Ping", 1910)))
          << " : " << logfiles_[16];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[17]),
                  UnorderedElementsAre(std::make_tuple(
                      "/pi2/aos", "aos.message_bridge.Timestamp", 9)))
          << " : " << logfiles_[17];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[18]),
                  UnorderedElementsAre(std::make_tuple(
                      "/pi2/aos", "aos.message_bridge.Timestamp", 191)))
          << " : " << logfiles_[18];
    } else {
      EXPECT_THAT(CountChannelsData(config, logfiles_[14]),
                  UnorderedElementsAre())
          << " : " << logfiles_[14];
      EXPECT_THAT(CountChannelsData(config, logfiles_[15]),
                  UnorderedElementsAre())
          << " : " << logfiles_[15];
      EXPECT_THAT(CountChannelsData(config, logfiles_[16]),
                  UnorderedElementsAre())
          << " : " << logfiles_[16];
      EXPECT_THAT(CountChannelsData(config, logfiles_[17]),
                  UnorderedElementsAre())
          << " : " << logfiles_[17];
      EXPECT_THAT(CountChannelsData(config, logfiles_[18]),
                  UnorderedElementsAre())
          << " : " << logfiles_[18];
      EXPECT_THAT(CountChannelsData(config, logfiles_[19]),
                  UnorderedElementsAre())
          << " : " << logfiles_[19];

      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[14]),
                  UnorderedElementsAre(std::make_tuple(
                      "/pi1/aos", "aos.message_bridge.Timestamp", 9)))
          << " : " << logfiles_[14];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[15]),
                  UnorderedElementsAre(std::make_tuple(
                      "/pi1/aos", "aos.message_bridge.Timestamp", 191)))
          << " : " << logfiles_[15];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[16]),
                  UnorderedElementsAre(std::make_tuple(
                      "/pi2/aos", "aos.message_bridge.Timestamp", 9)))
          << " : " << logfiles_[16];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[17]),
                  UnorderedElementsAre(std::make_tuple(
                      "/pi2/aos", "aos.message_bridge.Timestamp", 191)))
          << " : " << logfiles_[17];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[18]),
                  UnorderedElementsAre(
                      std::make_tuple("/test", "aos.examples.Ping", 91)))
          << " : " << logfiles_[18];
      EXPECT_THAT(CountChannelsTimestamp(config, logfiles_[19]),
                  UnorderedElementsAre(
                      std::make_tuple("/test", "aos.examples.Ping", 1910)))
          << " : " << logfiles_[19];
    }
  }

  LogReader reader(sorted_log_files);

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi1) << " pi1";
  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi2) << " pi2";
  LOG(INFO) << "now pi1 "
            << log_reader_factory.GetNodeEventLoopFactory(pi1)->monotonic_now();
  LOG(INFO) << "now pi2 "
            << log_reader_factory.GetNodeEventLoopFactory(pi2)->monotonic_now();

  EXPECT_THAT(reader.LoggedNodes(),
              ::testing::ElementsAre(
                  configuration::GetNode(reader.logged_configuration(), pi1),
                  configuration::GetNode(reader.logged_configuration(), pi2)));

  reader.event_loop_factory()->set_send_delay(chrono::microseconds(0));

  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);

  int pi1_ping_count = 10;
  int pi2_ping_count = 10;
  int pi1_pong_count = 10;
  int pi2_pong_count = 10;

  // Confirm that the ping value matches.
  pi1_event_loop->MakeWatcher(
      "/test", [&pi1_ping_count, &pi1_event_loop](const examples::Ping &ping) {
        VLOG(1) << "Pi1 ping " << FlatbufferToJson(&ping) << " at "
                << pi1_event_loop->context().monotonic_remote_time << " -> "
                << pi1_event_loop->context().monotonic_event_time;
        EXPECT_EQ(ping.value(), pi1_ping_count + 1);
        EXPECT_EQ(pi1_event_loop->context().monotonic_remote_time,
                  pi1_ping_count * chrono::milliseconds(10) +
                      monotonic_clock::epoch());
        EXPECT_EQ(pi1_event_loop->context().realtime_remote_time,
                  pi1_ping_count * chrono::milliseconds(10) +
                      realtime_clock::epoch());
        EXPECT_EQ(pi1_event_loop->context().monotonic_remote_time,
                  pi1_event_loop->context().monotonic_event_time);
        EXPECT_EQ(pi1_event_loop->context().realtime_remote_time,
                  pi1_event_loop->context().realtime_event_time);

        ++pi1_ping_count;
      });
  pi2_event_loop->MakeWatcher(
      "/test", [&pi2_ping_count, &pi2_event_loop](const examples::Ping &ping) {
        VLOG(1) << "Pi2 ping " << FlatbufferToJson(&ping) << " at "
                << pi2_event_loop->context().monotonic_remote_time << " -> "
                << pi2_event_loop->context().monotonic_event_time;
        EXPECT_EQ(ping.value(), pi2_ping_count + 1);

        EXPECT_EQ(pi2_event_loop->context().monotonic_remote_time,
                  pi2_ping_count * chrono::milliseconds(10) +
                      monotonic_clock::epoch());
        EXPECT_EQ(pi2_event_loop->context().realtime_remote_time,
                  pi2_ping_count * chrono::milliseconds(10) +
                      realtime_clock::epoch());
        EXPECT_EQ(pi2_event_loop->context().monotonic_remote_time +
                      chrono::microseconds(150),
                  pi2_event_loop->context().monotonic_event_time);
        EXPECT_EQ(pi2_event_loop->context().realtime_remote_time +
                      chrono::microseconds(150),
                  pi2_event_loop->context().realtime_event_time);
        ++pi2_ping_count;
      });

  constexpr ssize_t kQueueIndexOffset = -9;
  // Confirm that the ping and pong counts both match, and the value also
  // matches.
  pi1_event_loop->MakeWatcher(
      "/test", [&pi1_event_loop, &pi1_ping_count,
                &pi1_pong_count](const examples::Pong &pong) {
        VLOG(1) << "Pi1 pong " << FlatbufferToJson(&pong) << " at "
                << pi1_event_loop->context().monotonic_remote_time << " -> "
                << pi1_event_loop->context().monotonic_event_time;

        EXPECT_EQ(pi1_event_loop->context().remote_queue_index,
                  pi1_pong_count + kQueueIndexOffset);
        EXPECT_EQ(pi1_event_loop->context().monotonic_remote_time,
                  chrono::microseconds(200) +
                      pi1_pong_count * chrono::milliseconds(10) +
                      monotonic_clock::epoch());
        EXPECT_EQ(pi1_event_loop->context().realtime_remote_time,
                  chrono::microseconds(200) +
                      pi1_pong_count * chrono::milliseconds(10) +
                      realtime_clock::epoch());

        EXPECT_EQ(pi1_event_loop->context().monotonic_remote_time +
                      chrono::microseconds(150),
                  pi1_event_loop->context().monotonic_event_time);
        EXPECT_EQ(pi1_event_loop->context().realtime_remote_time +
                      chrono::microseconds(150),
                  pi1_event_loop->context().realtime_event_time);

        EXPECT_EQ(pong.value(), pi1_pong_count + 1);
        ++pi1_pong_count;
        EXPECT_EQ(pi1_ping_count, pi1_pong_count);
      });
  pi2_event_loop->MakeWatcher(
      "/test", [&pi2_event_loop, &pi2_ping_count,
                &pi2_pong_count](const examples::Pong &pong) {
        VLOG(1) << "Pi2 pong " << FlatbufferToJson(&pong) << " at "
                << pi2_event_loop->context().monotonic_remote_time << " -> "
                << pi2_event_loop->context().monotonic_event_time;

        EXPECT_EQ(pi2_event_loop->context().remote_queue_index,
                  pi2_pong_count + kQueueIndexOffset);

        EXPECT_EQ(pi2_event_loop->context().monotonic_remote_time,
                  chrono::microseconds(200) +
                      pi2_pong_count * chrono::milliseconds(10) +
                      monotonic_clock::epoch());
        EXPECT_EQ(pi2_event_loop->context().realtime_remote_time,
                  chrono::microseconds(200) +
                      pi2_pong_count * chrono::milliseconds(10) +
                      realtime_clock::epoch());

        EXPECT_EQ(pi2_event_loop->context().monotonic_remote_time,
                  pi2_event_loop->context().monotonic_event_time);
        EXPECT_EQ(pi2_event_loop->context().realtime_remote_time,
                  pi2_event_loop->context().realtime_event_time);

        EXPECT_EQ(pong.value(), pi2_pong_count + 1);
        ++pi2_pong_count;
        EXPECT_EQ(pi2_ping_count, pi2_pong_count);
      });

  log_reader_factory.Run();
  EXPECT_EQ(pi1_ping_count, 2010);
  EXPECT_EQ(pi2_ping_count, 2010);
  EXPECT_EQ(pi1_pong_count, 2010);
  EXPECT_EQ(pi2_pong_count, 2010);

  reader.Deregister();
}

// Test that if we feed the replay with a mismatched node list that we die on
// the LogReader constructor.
TEST_P(MultinodeLoggerDeathTest, MultiNodeBadReplayConfig) {
  time_converter_.StartEqual();
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  // Test that, if we add an additional node to the replay config that the
  // logger complains about the mismatch in number of nodes.
  FlatbufferDetachedBuffer<Configuration> extra_nodes_config =
      configuration::MergeWithConfig(&config_.message(), R"({
          "nodes": [
            {
              "name": "extra-node"
            }
          ]
        }
      )");

  const std::vector<LogFile> sorted_parts = SortParts(logfiles_);
  EXPECT_DEATH(LogReader(sorted_parts, &extra_nodes_config.message()),
               "Log file and replay config need to have matching nodes lists.");
}

// Tests that we can read log files where they don't start at the same monotonic
// time.
TEST_P(MultinodeLoggerTest, StaggeredStart) {
  time_converter_.StartEqual();
  std::vector<std::string> actual_filenames;

  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(200));

    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
    pi1_logger.AppendAllFilenames(&actual_filenames);
    pi2_logger.AppendAllFilenames(&actual_filenames);
  }

  // Since we delay starting pi2, it already knows about all the timestamps so
  // we don't end up with extra parts.
  LogReader reader(SortParts(actual_filenames));

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  EXPECT_THAT(reader.LoggedNodes(),
              ::testing::ElementsAre(
                  configuration::GetNode(reader.logged_configuration(), pi1),
                  configuration::GetNode(reader.logged_configuration(), pi2)));

  reader.event_loop_factory()->set_send_delay(chrono::microseconds(0));

  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);

  int pi1_ping_count = 30;
  int pi2_ping_count = 30;
  int pi1_pong_count = 30;
  int pi2_pong_count = 30;

  // Confirm that the ping value matches.
  pi1_event_loop->MakeWatcher(
      "/test", [&pi1_ping_count, &pi1_event_loop](const examples::Ping &ping) {
        VLOG(1) << "Pi1 ping " << FlatbufferToJson(&ping)
                << pi1_event_loop->context().monotonic_remote_time << " -> "
                << pi1_event_loop->context().monotonic_event_time;
        EXPECT_EQ(ping.value(), pi1_ping_count + 1);

        ++pi1_ping_count;
      });
  pi2_event_loop->MakeWatcher(
      "/test", [&pi2_ping_count, &pi2_event_loop](const examples::Ping &ping) {
        VLOG(1) << "Pi2 ping " << FlatbufferToJson(&ping)
                << pi2_event_loop->context().monotonic_remote_time << " -> "
                << pi2_event_loop->context().monotonic_event_time;
        EXPECT_EQ(ping.value(), pi2_ping_count + 1);

        ++pi2_ping_count;
      });

  // Confirm that the ping and pong counts both match, and the value also
  // matches.
  pi1_event_loop->MakeWatcher(
      "/test", [&pi1_event_loop, &pi1_ping_count,
                &pi1_pong_count](const examples::Pong &pong) {
        VLOG(1) << "Pi1 pong " << FlatbufferToJson(&pong) << " at "
                << pi1_event_loop->context().monotonic_remote_time << " -> "
                << pi1_event_loop->context().monotonic_event_time;

        EXPECT_EQ(pong.value(), pi1_pong_count + 1);
        ++pi1_pong_count;
        EXPECT_EQ(pi1_ping_count, pi1_pong_count);
      });
  pi2_event_loop->MakeWatcher(
      "/test", [&pi2_event_loop, &pi2_ping_count,
                &pi2_pong_count](const examples::Pong &pong) {
        VLOG(1) << "Pi2 pong " << FlatbufferToJson(&pong) << " at "
                << pi2_event_loop->context().monotonic_remote_time << " -> "
                << pi2_event_loop->context().monotonic_event_time;

        EXPECT_EQ(pong.value(), pi2_pong_count + 1);
        ++pi2_pong_count;
        EXPECT_EQ(pi2_ping_count, pi2_pong_count);
      });

  log_reader_factory.Run();
  EXPECT_EQ(pi1_ping_count, 2030);
  EXPECT_EQ(pi2_ping_count, 2030);
  EXPECT_EQ(pi1_pong_count, 2030);
  EXPECT_EQ(pi2_pong_count, 2030);

  reader.Deregister();
}

// Tests that we can read log files where the monotonic clocks drift and don't
// match correctly.  While we are here, also test that different ending times
// also is readable.
TEST_P(MultinodeLoggerTest, MismatchedClocks) {
  // TODO(austin): Negate...
  const chrono::nanoseconds initial_pi2_offset = chrono::seconds(1000);

  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(), BootTimestamp::epoch() + initial_pi2_offset});
  // Wait for 95 ms, (~0.1 seconds - 1/2 of the ping/pong period), and set the
  // skew to be 200 uS/s
  const chrono::nanoseconds startup_sleep1 = time_converter_.AddMonotonic(
      {chrono::milliseconds(95),
       chrono::milliseconds(95) - chrono::nanoseconds(200) * 95});
  // Run another 200 ms to have one logger start first.
  const chrono::nanoseconds startup_sleep2 = time_converter_.AddMonotonic(
      {chrono::milliseconds(200), chrono::milliseconds(200)});
  // Slew one way then the other at the same 200 uS/S slew rate.  Make sure we
  // go far enough to cause problems if this isn't accounted for.
  const chrono::nanoseconds logger_run1 = time_converter_.AddMonotonic(
      {chrono::milliseconds(20000),
       chrono::milliseconds(20000) - chrono::nanoseconds(200) * 20000});
  const chrono::nanoseconds logger_run2 = time_converter_.AddMonotonic(
      {chrono::milliseconds(40000),
       chrono::milliseconds(40000) + chrono::nanoseconds(200) * 40000});
  const chrono::nanoseconds logger_run3 = time_converter_.AddMonotonic(
      {chrono::milliseconds(400), chrono::milliseconds(400)});

  {
    LoggerState pi2_logger = MakeLogger(pi2_);

    LOG(INFO) << "pi2 times: " << pi2_->monotonic_now() << " "
              << pi2_->realtime_now() << " distributed "
              << pi2_->ToDistributedClock(pi2_->monotonic_now());

    LOG(INFO) << "pi2_ times: " << pi2_->monotonic_now() << " "
              << pi2_->realtime_now() << " distributed "
              << pi2_->ToDistributedClock(pi2_->monotonic_now());

    event_loop_factory_.RunFor(startup_sleep1);

    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(startup_sleep2);

    {
      // Run pi1's logger for only part of the time.
      LoggerState pi1_logger = MakeLogger(pi1_);

      StartLogger(&pi1_logger);
      event_loop_factory_.RunFor(logger_run1);

      // Make sure we slewed time far enough so that the difference is greater
      // than the network delay.  This confirms that if we sort incorrectly, it
      // would show in the results.
      EXPECT_LT(
          (pi2_->monotonic_now() - pi1_->monotonic_now()) - initial_pi2_offset,
          -event_loop_factory_.send_delay() -
              event_loop_factory_.network_delay());

      event_loop_factory_.RunFor(logger_run2);

      // And now check that we went far enough the other way to make sure we
      // cover both problems.
      EXPECT_GT(
          (pi2_->monotonic_now() - pi1_->monotonic_now()) - initial_pi2_offset,
          event_loop_factory_.send_delay() +
              event_loop_factory_.network_delay());
    }

    // And log a bit more on pi2.
    event_loop_factory_.RunFor(logger_run3);
  }

  LogReader reader(
      SortParts(MakeLogFiles(logfile_base1_, logfile_base2_, 3, 2)));

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi1) << " pi1";
  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi2) << " pi2";
  LOG(INFO) << "now pi1 "
            << log_reader_factory.GetNodeEventLoopFactory(pi1)->monotonic_now();
  LOG(INFO) << "now pi2 "
            << log_reader_factory.GetNodeEventLoopFactory(pi2)->monotonic_now();

  LOG(INFO) << "Done registering (pi1) "
            << log_reader_factory.GetNodeEventLoopFactory(pi1)->monotonic_now()
            << " "
            << log_reader_factory.GetNodeEventLoopFactory(pi1)->realtime_now();
  LOG(INFO) << "Done registering (pi2) "
            << log_reader_factory.GetNodeEventLoopFactory(pi2)->monotonic_now()
            << " "
            << log_reader_factory.GetNodeEventLoopFactory(pi2)->realtime_now();

  EXPECT_THAT(reader.LoggedNodes(),
              ::testing::ElementsAre(
                  configuration::GetNode(reader.logged_configuration(), pi1),
                  configuration::GetNode(reader.logged_configuration(), pi2)));

  reader.event_loop_factory()->set_send_delay(chrono::microseconds(0));

  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);

  int pi1_ping_count = 30;
  int pi2_ping_count = 30;
  int pi1_pong_count = 30;
  int pi2_pong_count = 30;

  // Confirm that the ping value matches.
  pi1_event_loop->MakeWatcher(
      "/test", [&pi1_ping_count, &pi1_event_loop](const examples::Ping &ping) {
        VLOG(1) << "Pi1 ping " << FlatbufferToJson(&ping)
                << pi1_event_loop->context().monotonic_remote_time << " -> "
                << pi1_event_loop->context().monotonic_event_time;
        EXPECT_EQ(ping.value(), pi1_ping_count + 1);

        ++pi1_ping_count;
      });
  pi2_event_loop->MakeWatcher(
      "/test", [&pi2_ping_count, &pi2_event_loop](const examples::Ping &ping) {
        VLOG(1) << "Pi2 ping " << FlatbufferToJson(&ping)
                << pi2_event_loop->context().monotonic_remote_time << " -> "
                << pi2_event_loop->context().monotonic_event_time;
        EXPECT_EQ(ping.value(), pi2_ping_count + 1);

        ++pi2_ping_count;
      });

  // Confirm that the ping and pong counts both match, and the value also
  // matches.
  pi1_event_loop->MakeWatcher(
      "/test", [&pi1_event_loop, &pi1_ping_count,
                &pi1_pong_count](const examples::Pong &pong) {
        VLOG(1) << "Pi1 pong " << FlatbufferToJson(&pong) << " at "
                << pi1_event_loop->context().monotonic_remote_time << " -> "
                << pi1_event_loop->context().monotonic_event_time;

        EXPECT_EQ(pong.value(), pi1_pong_count + 1);
        ++pi1_pong_count;
        EXPECT_EQ(pi1_ping_count, pi1_pong_count);
      });
  pi2_event_loop->MakeWatcher(
      "/test", [&pi2_event_loop, &pi2_ping_count,
                &pi2_pong_count](const examples::Pong &pong) {
        VLOG(1) << "Pi2 pong " << FlatbufferToJson(&pong) << " at "
                << pi2_event_loop->context().monotonic_remote_time << " -> "
                << pi2_event_loop->context().monotonic_event_time;

        EXPECT_EQ(pong.value(), pi2_pong_count + 1);
        ++pi2_pong_count;
        EXPECT_EQ(pi2_ping_count, pi2_pong_count);
      });

  log_reader_factory.Run();
  EXPECT_EQ(pi1_ping_count, 6030);
  EXPECT_EQ(pi2_ping_count, 6030);
  EXPECT_EQ(pi1_pong_count, 6030);
  EXPECT_EQ(pi2_pong_count, 6030);

  reader.Deregister();
}

// Tests that we can sort a bunch of parts into the pre-determined sorted parts.
TEST_P(MultinodeLoggerTest, SortParts) {
  time_converter_.StartEqual();
  // Make a bunch of parts.
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(2000));
  }

  const std::vector<LogFile> sorted_parts = SortParts(logfiles_);
  VerifyParts(sorted_parts);
}

// Tests that we can sort a bunch of parts with an empty part.  We should ignore
// it and remove it from the sorted list.
TEST_P(MultinodeLoggerTest, SortEmptyParts) {
  time_converter_.StartEqual();
  // Make a bunch of parts.
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(2000));
  }

  // TODO(austin): Should we flip out if the file can't open?
  const std::string kEmptyFile("foobarinvalidfiledoesnotexist" + Extension());

  aos::util::WriteStringToFileOrDie(kEmptyFile, "");
  logfiles_.emplace_back(kEmptyFile);

  const std::vector<LogFile> sorted_parts = SortParts(logfiles_);
  VerifyParts(sorted_parts, {kEmptyFile});
}

// Tests that we can sort a bunch of parts with the end missing off a
// file.  We should use the part we can read.
TEST_P(MultinodeLoggerTest, SortTruncatedParts) {
  std::vector<std::string> actual_filenames;
  time_converter_.StartEqual();
  // Make a bunch of parts.
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(2000));

    pi1_logger.AppendAllFilenames(&actual_filenames);
    pi2_logger.AppendAllFilenames(&actual_filenames);
  }

  ASSERT_THAT(actual_filenames,
              ::testing::UnorderedElementsAreArray(logfiles_));

  // Strip off the end of one of the files.  Pick one with a lot of data.
  // For snappy, needs to have enough data to be >1 chunk of compressed data so
  // that we don't corrupt the entire log part.
  ::std::string compressed_contents =
      aos::util::ReadFileToStringOrDie(logfiles_[4]);

  aos::util::WriteStringToFileOrDie(
      logfiles_[4],
      compressed_contents.substr(0, compressed_contents.size() - 100));

  const std::vector<LogFile> sorted_parts = SortParts(logfiles_);
  VerifyParts(sorted_parts);
}

// Tests that if we remap a logged channel, it shows up correctly.
TEST_P(MultinodeLoggerTest, RemapLoggedChannel) {
  time_converter_.StartEqual();
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  LogReader reader(SortParts(logfiles_));

  // Remap just on pi1.
  reader.RemapLoggedChannel<aos::timing::Report>(
      "/aos", configuration::GetNode(reader.configuration(), "pi1"));

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  std::vector<const Channel *> remapped_channels = reader.RemappedChannels();
  // Note: An extra channel gets remapped automatically due to a timestamp
  // channel being LOCAL_LOGGER'd.
  ASSERT_EQ(remapped_channels.size(), std::get<0>(GetParam()).shared ? 1u : 2u);
  EXPECT_EQ(remapped_channels[0]->name()->string_view(), "/original/pi1/aos");
  EXPECT_EQ(remapped_channels[0]->type()->string_view(), "aos.timing.Report");
  if (!std::get<0>(GetParam()).shared) {
    EXPECT_EQ(remapped_channels[1]->name()->string_view(),
              "/original/pi1/aos/remote_timestamps/pi2/pi1/aos/"
              "aos-message_bridge-Timestamp");
    EXPECT_EQ(remapped_channels[1]->type()->string_view(),
              "aos.message_bridge.RemoteMessage");
  }

  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  // Confirm we can read the data on the remapped channel, just for pi1. Nothing
  // else should have moved.
  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  pi1_event_loop->SkipTimingReport();
  std::unique_ptr<EventLoop> full_pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  full_pi1_event_loop->SkipTimingReport();
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);
  pi2_event_loop->SkipTimingReport();

  MessageCounter<aos::timing::Report> pi1_timing_report(pi1_event_loop.get(),
                                                        "/aos");
  MessageCounter<aos::timing::Report> full_pi1_timing_report(
      full_pi1_event_loop.get(), "/pi1/aos");
  MessageCounter<aos::timing::Report> pi1_original_timing_report(
      pi1_event_loop.get(), "/original/aos");
  MessageCounter<aos::timing::Report> full_pi1_original_timing_report(
      full_pi1_event_loop.get(), "/original/pi1/aos");
  MessageCounter<aos::timing::Report> pi2_timing_report(pi2_event_loop.get(),
                                                        "/aos");

  log_reader_factory.Run();

  EXPECT_EQ(pi1_timing_report.count(), 0u);
  EXPECT_EQ(full_pi1_timing_report.count(), 0u);
  EXPECT_NE(pi1_original_timing_report.count(), 0u);
  EXPECT_NE(full_pi1_original_timing_report.count(), 0u);
  EXPECT_NE(pi2_timing_report.count(), 0u);

  reader.Deregister();
}

// Tests that if we rename a logged channel, it shows up correctly.
TEST_P(MultinodeLoggerTest, RenameLoggedChannel) {
  std::vector<std::string> actual_filenames;
  time_converter_.StartEqual();
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));

    pi1_logger.AppendAllFilenames(&actual_filenames);
    pi2_logger.AppendAllFilenames(&actual_filenames);
  }

  LogReader reader(SortParts(actual_filenames));

  // Rename just on pi2. Add some global maps just to verify they get added in
  // the config and used correctly.
  std::vector<MapT> maps;
  {
    MapT map;
    map.match = std::make_unique<ChannelT>();
    map.match->name = "/foo*";
    map.match->source_node = "pi1";
    map.rename = std::make_unique<ChannelT>();
    map.rename->name = "/pi1/foo";
    maps.emplace_back(std::move(map));
  }
  {
    MapT map;
    map.match = std::make_unique<ChannelT>();
    map.match->name = "/foo*";
    map.match->source_node = "pi2";
    map.rename = std::make_unique<ChannelT>();
    map.rename->name = "/pi2/foo";
    maps.emplace_back(std::move(map));
  }
  {
    MapT map;
    map.match = std::make_unique<ChannelT>();
    map.match->name = "/foo";
    map.match->type = "aos.examples.Ping";
    map.rename = std::make_unique<ChannelT>();
    map.rename->name = "/foo/renamed";
    maps.emplace_back(std::move(map));
  }
  reader.RenameLoggedChannel<aos::examples::Ping>(
      "/aos", configuration::GetNode(reader.configuration(), "pi2"),
      "/pi2/foo/renamed", maps);

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  std::vector<const Channel *> remapped_channels = reader.RemappedChannels();
  // Note: An extra channel gets remapped automatically due to a timestamp
  // channel being LOCAL_LOGGER'd.
  const bool shared = std::get<0>(GetParam()).shared;
  ASSERT_EQ(remapped_channels.size(), shared ? 1u : 2u);
  EXPECT_EQ(remapped_channels[shared ? 0 : 1]->name()->string_view(),
            "/pi2/foo/renamed");
  EXPECT_EQ(remapped_channels[shared ? 0 : 1]->type()->string_view(),
            "aos.examples.Ping");
  if (!shared) {
    EXPECT_EQ(remapped_channels[0]->name()->string_view(),
              "/original/pi1/aos/remote_timestamps/pi2/pi1/aos/"
              "aos-message_bridge-Timestamp");
    EXPECT_EQ(remapped_channels[0]->type()->string_view(),
              "aos.message_bridge.RemoteMessage");
  }

  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  // Confirm we can read the data on the renamed channel, just for pi2. Nothing
  // else should have moved.
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);
  pi2_event_loop->SkipTimingReport();
  std::unique_ptr<EventLoop> full_pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);
  full_pi2_event_loop->SkipTimingReport();
  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  pi1_event_loop->SkipTimingReport();

  MessageCounter<aos::examples::Ping> pi2_ping(pi2_event_loop.get(), "/aos");
  MessageCounter<aos::examples::Ping> pi2_renamed_ping(pi2_event_loop.get(),
                                                       "/foo");
  MessageCounter<aos::examples::Ping> full_pi2_renamed_ping(
      full_pi2_event_loop.get(), "/pi2/foo/renamed");
  MessageCounter<aos::examples::Ping> pi1_ping(pi1_event_loop.get(), "/aos");

  log_reader_factory.Run();

  EXPECT_EQ(pi2_ping.count(), 0u);
  EXPECT_NE(pi2_renamed_ping.count(), 0u);
  EXPECT_NE(full_pi2_renamed_ping.count(), 0u);
  EXPECT_NE(pi1_ping.count(), 0u);

  reader.Deregister();
}

// Tests that we can remap a forwarded channel as well.
TEST_P(MultinodeLoggerTest, RemapForwardedLoggedChannel) {
  time_converter_.StartEqual();
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  LogReader reader(SortParts(logfiles_));

  reader.RemapLoggedChannel<examples::Ping>("/test");

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  // Confirm we can read the data on the remapped channel, just for pi1. Nothing
  // else should have moved.
  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  pi1_event_loop->SkipTimingReport();
  std::unique_ptr<EventLoop> full_pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  full_pi1_event_loop->SkipTimingReport();
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);
  pi2_event_loop->SkipTimingReport();

  MessageCounter<examples::Ping> pi1_ping(pi1_event_loop.get(), "/test");
  MessageCounter<examples::Ping> pi2_ping(pi2_event_loop.get(), "/test");
  MessageCounter<examples::Ping> pi1_original_ping(pi1_event_loop.get(),
                                                   "/original/test");
  MessageCounter<examples::Ping> pi2_original_ping(pi2_event_loop.get(),
                                                   "/original/test");

  std::unique_ptr<MessageCounter<message_bridge::RemoteMessage>>
      pi1_original_ping_timestamp;
  std::unique_ptr<MessageCounter<message_bridge::RemoteMessage>>
      pi1_ping_timestamp;
  if (!shared()) {
    pi1_original_ping_timestamp =
        std::make_unique<MessageCounter<message_bridge::RemoteMessage>>(
            pi1_event_loop.get(),
            "/pi1/aos/remote_timestamps/pi2/original/test/aos-examples-Ping");
    pi1_ping_timestamp =
        std::make_unique<MessageCounter<message_bridge::RemoteMessage>>(
            pi1_event_loop.get(),
            "/pi1/aos/remote_timestamps/pi2/test/aos-examples-Ping");
  }

  log_reader_factory.Run();

  EXPECT_EQ(pi1_ping.count(), 0u);
  EXPECT_EQ(pi2_ping.count(), 0u);
  EXPECT_NE(pi1_original_ping.count(), 0u);
  EXPECT_NE(pi2_original_ping.count(), 0u);
  if (!shared()) {
    EXPECT_NE(pi1_original_ping_timestamp->count(), 0u);
    EXPECT_EQ(pi1_ping_timestamp->count(), 0u);
  }

  reader.Deregister();
}

// Tests that we can rename a forwarded channel as well.
TEST_P(MultinodeLoggerTest, RenameForwardedLoggedChannel) {
  std::vector<std::string> actual_filenames;
  time_converter_.StartEqual();
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));

    pi1_logger.AppendAllFilenames(&actual_filenames);
    pi2_logger.AppendAllFilenames(&actual_filenames);
  }

  LogReader reader(SortParts(actual_filenames));

  std::vector<MapT> maps;
  {
    MapT map;
    map.match = std::make_unique<ChannelT>();
    map.match->name = "/production*";
    map.match->source_node = "pi1";
    map.rename = std::make_unique<ChannelT>();
    map.rename->name = "/pi1/production";
    maps.emplace_back(std::move(map));
  }
  {
    MapT map;
    map.match = std::make_unique<ChannelT>();
    map.match->name = "/production*";
    map.match->source_node = "pi2";
    map.rename = std::make_unique<ChannelT>();
    map.rename->name = "/pi2/production";
    maps.emplace_back(std::move(map));
  }
  reader.RenameLoggedChannel<aos::examples::Ping>(
      "/test", configuration::GetNode(reader.configuration(), "pi1"),
      "/pi1/production", maps);

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  // Confirm we can read the data on the renamed channel, on both the source
  // node and the remote node. In case of split timestamp channels, confirm that
  // we receive the timestamp messages on the renamed channel as well.
  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  pi1_event_loop->SkipTimingReport();
  std::unique_ptr<EventLoop> full_pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  full_pi1_event_loop->SkipTimingReport();
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);
  pi2_event_loop->SkipTimingReport();

  MessageCounter<examples::Ping> pi1_ping(pi1_event_loop.get(), "/test");
  MessageCounter<examples::Ping> pi2_ping(pi2_event_loop.get(), "/test");
  MessageCounter<examples::Ping> pi1_renamed_ping(pi1_event_loop.get(),
                                                  "/pi1/production");
  MessageCounter<examples::Ping> pi2_renamed_ping(pi2_event_loop.get(),
                                                  "/pi1/production");

  std::unique_ptr<MessageCounter<message_bridge::RemoteMessage>>
      pi1_renamed_ping_timestamp;
  std::unique_ptr<MessageCounter<message_bridge::RemoteMessage>>
      pi1_ping_timestamp;
  if (!shared()) {
    pi1_renamed_ping_timestamp =
        std::make_unique<MessageCounter<message_bridge::RemoteMessage>>(
            pi1_event_loop.get(),
            "/pi1/aos/remote_timestamps/pi2/pi1/production/aos-examples-Ping");
    pi1_ping_timestamp =
        std::make_unique<MessageCounter<message_bridge::RemoteMessage>>(
            pi1_event_loop.get(),
            "/pi1/aos/remote_timestamps/pi2/test/aos-examples-Ping");
  }

  log_reader_factory.Run();

  EXPECT_EQ(pi1_ping.count(), 0u);
  EXPECT_EQ(pi2_ping.count(), 0u);
  EXPECT_NE(pi1_renamed_ping.count(), 0u);
  EXPECT_NE(pi2_renamed_ping.count(), 0u);
  if (!shared()) {
    EXPECT_NE(pi1_renamed_ping_timestamp->count(), 0u);
    EXPECT_EQ(pi1_ping_timestamp->count(), 0u);
  }

  reader.Deregister();
}

// Tests that we observe all the same events in log replay (for a given node)
// whether we just register an event loop for that node or if we register a full
// event loop factory.
TEST_P(MultinodeLoggerTest, SingleNodeReplay) {
  time_converter_.StartEqual();
  constexpr chrono::milliseconds kStartupDelay(95);
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(kStartupDelay);

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  LogReader full_reader(SortParts(logfiles_));
  LogReader single_node_reader(SortParts(logfiles_));

  SimulatedEventLoopFactory full_factory(full_reader.configuration());
  SimulatedEventLoopFactory single_node_factory(
      single_node_reader.configuration());
  single_node_factory.SkipTimingReport();
  single_node_factory.DisableStatistics();
  std::unique_ptr<EventLoop> replay_event_loop =
      single_node_factory.GetNodeEventLoopFactory("pi1")->MakeEventLoop(
          "log_reader");

  full_reader.Register(&full_factory);
  single_node_reader.Register(replay_event_loop.get());

  const Node *full_pi1 =
      configuration::GetNode(full_factory.configuration(), "pi1");

  // Confirm we can read the data on the remapped channel, just for pi1. Nothing
  // else should have moved.
  std::unique_ptr<EventLoop> full_event_loop =
      full_factory.MakeEventLoop("test", full_pi1);
  full_event_loop->SkipTimingReport();
  full_event_loop->SkipAosLog();
  // maps are indexed on channel index.
  // observed_messages: {channel_index: [(message_sent_time, was_fetched),...]}
  std::map<size_t, std::vector<std::pair<monotonic_clock::time_point, bool>>>
      observed_messages;
  std::map<size_t, std::unique_ptr<RawFetcher>> fetchers;
  for (size_t ii = 0; ii < full_event_loop->configuration()->channels()->size();
       ++ii) {
    const Channel *channel =
        full_event_loop->configuration()->channels()->Get(ii);
    // We currently don't support replaying remote timestamp channels in
    // realtime replay (unless the remote timestamp channel was not NOT_LOGGED,
    // in which case it gets auto-remapped and replayed on a /original channel).
    if (channel->name()->string_view().find("remote_timestamp") !=
            std::string_view::npos &&
        channel->name()->string_view().find("/original") ==
            std::string_view::npos) {
      continue;
    }
    if (configuration::ChannelIsReadableOnNode(channel, full_pi1)) {
      observed_messages[ii] = {};
      fetchers[ii] = full_event_loop->MakeRawFetcher(channel);
      full_event_loop->OnRun([ii, &observed_messages, &fetchers]() {
        if (fetchers[ii]->Fetch()) {
          observed_messages[ii].push_back(std::make_pair(
              fetchers[ii]->context().monotonic_event_time, true));
        }
      });
      full_event_loop->MakeRawNoArgWatcher(
          channel, [ii, &observed_messages](const Context &context) {
            observed_messages[ii].push_back(
                std::make_pair(context.monotonic_event_time, false));
          });
    }
  }

  full_factory.Run();
  fetchers.clear();
  full_reader.Deregister();

  const Node *single_node_pi1 =
      configuration::GetNode(single_node_factory.configuration(), "pi1");
  std::map<size_t, std::unique_ptr<RawFetcher>> single_node_fetchers;

  std::unique_ptr<EventLoop> single_node_event_loop =
      single_node_factory.MakeEventLoop("test", single_node_pi1);
  single_node_event_loop->SkipTimingReport();
  single_node_event_loop->SkipAosLog();
  for (size_t ii = 0;
       ii < single_node_event_loop->configuration()->channels()->size(); ++ii) {
    const Channel *channel =
        single_node_event_loop->configuration()->channels()->Get(ii);
    single_node_factory.DisableForwarding(channel);
    if (configuration::ChannelIsReadableOnNode(channel, single_node_pi1)) {
      single_node_fetchers[ii] =
          single_node_event_loop->MakeRawFetcher(channel);
      single_node_event_loop->OnRun([channel, ii, &single_node_fetchers]() {
        EXPECT_FALSE(single_node_fetchers[ii]->Fetch())
            << "Single EventLoop replay doesn't support pre-loading fetchers. "
            << configuration::StrippedChannelToString(channel);
      });
      single_node_event_loop->MakeRawNoArgWatcher(
          channel, [ii, &observed_messages, channel,
                    kStartupDelay](const Context &context) {
            if (observed_messages[ii].empty()) {
              FAIL() << "Observed extra message at "
                     << context.monotonic_event_time << " on "
                     << configuration::StrippedChannelToString(channel);
              return;
            }
            const std::pair<monotonic_clock::time_point, bool> &message =
                observed_messages[ii].front();
            if (message.second) {
              EXPECT_LE(message.first,
                        context.monotonic_event_time + kStartupDelay)
                  << "Mismatched message times " << context.monotonic_event_time
                  << " and " << message.first << " on "
                  << configuration::StrippedChannelToString(channel);
            } else {
              EXPECT_EQ(message.first,
                        context.monotonic_event_time + kStartupDelay)
                  << "Mismatched message times " << context.monotonic_event_time
                  << " and " << message.first << " on "
                  << configuration::StrippedChannelToString(channel);
            }
            observed_messages[ii].erase(observed_messages[ii].begin());
          });
    }
  }

  single_node_factory.Run();

  single_node_fetchers.clear();

  single_node_reader.Deregister();

  for (const auto &pair : observed_messages) {
    EXPECT_TRUE(pair.second.empty())
        << "Missed " << pair.second.size() << " messages on "
        << configuration::StrippedChannelToString(
               single_node_event_loop->configuration()->channels()->Get(
                   pair.first));
  }
}

// Tests that we properly recreate forwarded timestamps when replaying a log.
// This should be enough that we can then re-run the logger and get a valid log
// back.
TEST_P(MultinodeLoggerTest, MessageHeader) {
  time_converter_.StartEqual();
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  LogReader reader(SortParts(logfiles_));

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi1) << " pi1";
  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi2) << " pi2";
  LOG(INFO) << "now pi1 "
            << log_reader_factory.GetNodeEventLoopFactory(pi1)->monotonic_now();
  LOG(INFO) << "now pi2 "
            << log_reader_factory.GetNodeEventLoopFactory(pi2)->monotonic_now();

  EXPECT_THAT(reader.LoggedNodes(),
              ::testing::ElementsAre(
                  configuration::GetNode(reader.logged_configuration(), pi1),
                  configuration::GetNode(reader.logged_configuration(), pi2)));

  reader.event_loop_factory()->set_send_delay(chrono::microseconds(0));

  std::unique_ptr<EventLoop> pi1_event_loop =
      log_reader_factory.MakeEventLoop("test", pi1);
  std::unique_ptr<EventLoop> pi2_event_loop =
      log_reader_factory.MakeEventLoop("test", pi2);

  aos::Fetcher<message_bridge::Timestamp> pi1_timestamp_on_pi1_fetcher =
      pi1_event_loop->MakeFetcher<message_bridge::Timestamp>("/pi1/aos");
  aos::Fetcher<message_bridge::Timestamp> pi1_timestamp_on_pi2_fetcher =
      pi2_event_loop->MakeFetcher<message_bridge::Timestamp>("/pi1/aos");

  aos::Fetcher<examples::Ping> ping_on_pi1_fetcher =
      pi1_event_loop->MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<examples::Ping> ping_on_pi2_fetcher =
      pi2_event_loop->MakeFetcher<examples::Ping>("/test");

  aos::Fetcher<message_bridge::Timestamp> pi2_timestamp_on_pi2_fetcher =
      pi2_event_loop->MakeFetcher<message_bridge::Timestamp>("/pi2/aos");
  aos::Fetcher<message_bridge::Timestamp> pi2_timestamp_on_pi1_fetcher =
      pi1_event_loop->MakeFetcher<message_bridge::Timestamp>("/pi2/aos");

  aos::Fetcher<examples::Pong> pong_on_pi2_fetcher =
      pi2_event_loop->MakeFetcher<examples::Pong>("/test");
  aos::Fetcher<examples::Pong> pong_on_pi1_fetcher =
      pi1_event_loop->MakeFetcher<examples::Pong>("/test");

  const size_t pi1_timestamp_channel = configuration::ChannelIndex(
      pi1_event_loop->configuration(), pi1_timestamp_on_pi1_fetcher.channel());
  const size_t ping_timestamp_channel = configuration::ChannelIndex(
      pi2_event_loop->configuration(), ping_on_pi2_fetcher.channel());

  const size_t pi2_timestamp_channel = configuration::ChannelIndex(
      pi2_event_loop->configuration(), pi2_timestamp_on_pi2_fetcher.channel());
  const size_t pong_timestamp_channel = configuration::ChannelIndex(
      pi1_event_loop->configuration(), pong_on_pi1_fetcher.channel());

  const chrono::nanoseconds network_delay = event_loop_factory_.network_delay();
  const chrono::nanoseconds send_delay = event_loop_factory_.send_delay();

  for (std::pair<int, std::string> channel :
       shared()
           ? std::vector<
                 std::pair<int, std::string>>{{-1,
                                               "/aos/remote_timestamps/pi2"}}
           : std::vector<std::pair<int, std::string>>{
                 {pi1_timestamp_channel,
                  "/aos/remote_timestamps/pi2/pi1/aos/"
                  "aos-message_bridge-Timestamp"},
                 {ping_timestamp_channel,
                  "/aos/remote_timestamps/pi2/test/aos-examples-Ping"}}) {
    pi1_event_loop->MakeWatcher(
        channel.second,
        [&pi1_event_loop, &pi2_event_loop, pi1_timestamp_channel,
         ping_timestamp_channel, &pi1_timestamp_on_pi1_fetcher,
         &pi1_timestamp_on_pi2_fetcher, &ping_on_pi1_fetcher,
         &ping_on_pi2_fetcher, network_delay, send_delay,
         channel_index = channel.first](const RemoteMessage &header) {
          const aos::monotonic_clock::time_point header_monotonic_sent_time(
              chrono::nanoseconds(header.monotonic_sent_time()));
          const aos::realtime_clock::time_point header_realtime_sent_time(
              chrono::nanoseconds(header.realtime_sent_time()));
          const aos::monotonic_clock::time_point header_monotonic_remote_time(
              chrono::nanoseconds(header.monotonic_remote_time()));
          const aos::realtime_clock::time_point header_realtime_remote_time(
              chrono::nanoseconds(header.realtime_remote_time()));

          if (channel_index != -1) {
            ASSERT_EQ(channel_index, header.channel_index());
          }

          const Context *pi1_context = nullptr;
          const Context *pi2_context = nullptr;

          if (header.channel_index() == pi1_timestamp_channel) {
            ASSERT_TRUE(pi1_timestamp_on_pi1_fetcher.FetchNext());
            ASSERT_TRUE(pi1_timestamp_on_pi2_fetcher.FetchNext());
            pi1_context = &pi1_timestamp_on_pi1_fetcher.context();
            pi2_context = &pi1_timestamp_on_pi2_fetcher.context();
          } else if (header.channel_index() == ping_timestamp_channel) {
            ASSERT_TRUE(ping_on_pi1_fetcher.FetchNext());
            ASSERT_TRUE(ping_on_pi2_fetcher.FetchNext());
            pi1_context = &ping_on_pi1_fetcher.context();
            pi2_context = &ping_on_pi2_fetcher.context();
          } else {
            LOG(FATAL) << "Unknown channel " << FlatbufferToJson(&header) << " "
                       << configuration::CleanedChannelToString(
                              pi1_event_loop->configuration()->channels()->Get(
                                  header.channel_index()));
          }

          ASSERT_TRUE(header.has_boot_uuid());
          EXPECT_EQ(UUID::FromVector(header.boot_uuid()),
                    pi2_event_loop->boot_uuid());

          EXPECT_EQ(pi1_context->queue_index, header.remote_queue_index());
          EXPECT_EQ(pi2_context->remote_queue_index,
                    header.remote_queue_index());
          EXPECT_EQ(pi2_context->queue_index, header.queue_index());

          EXPECT_EQ(pi2_context->monotonic_event_time,
                    header_monotonic_sent_time);
          EXPECT_EQ(pi2_context->realtime_event_time,
                    header_realtime_sent_time);
          EXPECT_EQ(pi2_context->realtime_remote_time,
                    header_realtime_remote_time);
          EXPECT_EQ(pi2_context->monotonic_remote_time,
                    header_monotonic_remote_time);

          EXPECT_EQ(pi1_context->realtime_event_time,
                    header_realtime_remote_time);
          EXPECT_EQ(pi1_context->monotonic_event_time,
                    header_monotonic_remote_time);

          // Time estimation isn't perfect, but we know the clocks were
          // identical when logged, so we know when this should have come back.
          // Confirm we got it when we expected.
          EXPECT_EQ(pi1_event_loop->context().monotonic_event_time,
                    pi1_context->monotonic_event_time + 2 * network_delay +
                        send_delay);
        });
  }
  for (std::pair<int, std::string> channel :
       shared()
           ? std::vector<
                 std::pair<int, std::string>>{{-1,
                                               "/aos/remote_timestamps/pi1"}}
           : std::vector<std::pair<int, std::string>>{
                 {pi2_timestamp_channel,
                  "/aos/remote_timestamps/pi1/pi2/aos/"
                  "aos-message_bridge-Timestamp"}}) {
    pi2_event_loop->MakeWatcher(
        channel.second,
        [&pi2_event_loop, &pi1_event_loop, pi2_timestamp_channel,
         pong_timestamp_channel, &pi2_timestamp_on_pi2_fetcher,
         &pi2_timestamp_on_pi1_fetcher, &pong_on_pi2_fetcher,
         &pong_on_pi1_fetcher, network_delay, send_delay,
         channel_index = channel.first](const RemoteMessage &header) {
          const aos::monotonic_clock::time_point header_monotonic_sent_time(
              chrono::nanoseconds(header.monotonic_sent_time()));
          const aos::realtime_clock::time_point header_realtime_sent_time(
              chrono::nanoseconds(header.realtime_sent_time()));
          const aos::monotonic_clock::time_point header_monotonic_remote_time(
              chrono::nanoseconds(header.monotonic_remote_time()));
          const aos::realtime_clock::time_point header_realtime_remote_time(
              chrono::nanoseconds(header.realtime_remote_time()));

          if (channel_index != -1) {
            ASSERT_EQ(channel_index, header.channel_index());
          }

          const Context *pi2_context = nullptr;
          const Context *pi1_context = nullptr;

          if (header.channel_index() == pi2_timestamp_channel) {
            ASSERT_TRUE(pi2_timestamp_on_pi2_fetcher.FetchNext());
            ASSERT_TRUE(pi2_timestamp_on_pi1_fetcher.FetchNext());
            pi2_context = &pi2_timestamp_on_pi2_fetcher.context();
            pi1_context = &pi2_timestamp_on_pi1_fetcher.context();
          } else if (header.channel_index() == pong_timestamp_channel) {
            ASSERT_TRUE(pong_on_pi2_fetcher.FetchNext());
            ASSERT_TRUE(pong_on_pi1_fetcher.FetchNext());
            pi2_context = &pong_on_pi2_fetcher.context();
            pi1_context = &pong_on_pi1_fetcher.context();
          } else {
            LOG(FATAL) << "Unknown channel " << FlatbufferToJson(&header) << " "
                       << configuration::CleanedChannelToString(
                              pi2_event_loop->configuration()->channels()->Get(
                                  header.channel_index()));
          }

          ASSERT_TRUE(header.has_boot_uuid());
          EXPECT_EQ(UUID::FromVector(header.boot_uuid()),
                    pi1_event_loop->boot_uuid());

          EXPECT_EQ(pi2_context->queue_index, header.remote_queue_index());
          EXPECT_EQ(pi1_context->remote_queue_index,
                    header.remote_queue_index());
          EXPECT_EQ(pi1_context->queue_index, header.queue_index());

          EXPECT_EQ(pi1_context->monotonic_event_time,
                    header_monotonic_sent_time);
          EXPECT_EQ(pi1_context->realtime_event_time,
                    header_realtime_sent_time);
          EXPECT_EQ(pi1_context->realtime_remote_time,
                    header_realtime_remote_time);
          EXPECT_EQ(pi1_context->monotonic_remote_time,
                    header_monotonic_remote_time);

          EXPECT_EQ(pi2_context->realtime_event_time,
                    header_realtime_remote_time);
          EXPECT_EQ(pi2_context->monotonic_event_time,
                    header_monotonic_remote_time);

          // Time estimation isn't perfect, but we know the clocks were
          // identical when logged, so we know when this should have come back.
          // Confirm we got it when we expected.
          EXPECT_EQ(pi2_event_loop->context().monotonic_event_time,
                    pi2_context->monotonic_event_time + 2 * network_delay +
                        send_delay);
        });
  }

  // And confirm we can re-create a log again, while checking the contents.
  {
    LoggerState pi1_logger = MakeLogger(
        log_reader_factory.GetNodeEventLoopFactory("pi1"), &log_reader_factory);
    LoggerState pi2_logger = MakeLogger(
        log_reader_factory.GetNodeEventLoopFactory("pi2"), &log_reader_factory);

    StartLogger(&pi1_logger, tmp_dir_ + "/relogged1");
    StartLogger(&pi2_logger, tmp_dir_ + "/relogged2");

    log_reader_factory.Run();
  }

  reader.Deregister();

  // And verify that we can run the LogReader over the relogged files without
  // hitting any fatal errors.
  {
    LogReader relogged_reader(SortParts(MakeLogFiles(
        tmp_dir_ + "/relogged1", tmp_dir_ + "/relogged2", 3, 3, true)));
    relogged_reader.Register();

    relogged_reader.event_loop_factory()->Run();
  }
  // And confirm that we can read the logged file using the reader's
  // configuration.
  {
    LogReader relogged_reader(
        SortParts(MakeLogFiles(tmp_dir_ + "/relogged1", tmp_dir_ + "/relogged2",
                               3, 3, true)),
        reader.configuration());
    relogged_reader.Register();

    relogged_reader.event_loop_factory()->Run();
  }
}

// Tests that we properly populate and extract the logger_start time by setting
// up a clock difference between 2 nodes and looking at the resulting parts.
TEST_P(MultinodeLoggerTest, LoggerStartTime) {
  std::vector<std::string> actual_filenames;
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(), BootTimestamp::epoch() + chrono::seconds(1000)});
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(10000));

    pi1_logger.AppendAllFilenames(&actual_filenames);
    pi2_logger.AppendAllFilenames(&actual_filenames);
  }

  ASSERT_THAT(actual_filenames,
              ::testing::UnorderedElementsAreArray(logfiles_));

  for (const LogFile &log_file : SortParts(logfiles_)) {
    for (const LogParts &log_part : log_file.parts) {
      if (log_part.node == log_file.logger_node) {
        EXPECT_EQ(log_part.logger_monotonic_start_time,
                  aos::monotonic_clock::min_time);
        EXPECT_EQ(log_part.logger_realtime_start_time,
                  aos::realtime_clock::min_time);
      } else {
        const chrono::seconds offset = log_file.logger_node == "pi1"
                                           ? -chrono::seconds(1000)
                                           : chrono::seconds(1000);
        EXPECT_EQ(log_part.logger_monotonic_start_time,
                  log_part.monotonic_start_time + offset);
        EXPECT_EQ(log_part.logger_realtime_start_time,
                  log_file.realtime_start_time +
                      (log_part.logger_monotonic_start_time -
                       log_file.monotonic_start_time));
      }
    }
  }
}

// Test that renaming the base, renames the folder.
TEST_P(MultinodeLoggerTest, LoggerRenameFolder) {
  util::UnlinkRecursive(tmp_dir_ + "/renamefolder");
  util::UnlinkRecursive(tmp_dir_ + "/new-good");
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(), BootTimestamp::epoch() + chrono::seconds(1000)});
  logfile_base1_ = tmp_dir_ + "/renamefolder/multi_logfile1";
  logfile_base2_ = tmp_dir_ + "/renamefolder/multi_logfile2";
  logfiles_ = MakeLogFiles(logfile_base1_, logfile_base2_);
  LoggerState pi1_logger = MakeLogger(pi1_);
  LoggerState pi2_logger = MakeLogger(pi2_);

  StartLogger(&pi1_logger);
  StartLogger(&pi2_logger);

  event_loop_factory_.RunFor(chrono::milliseconds(10000));
  logfile_base1_ = tmp_dir_ + "/new-good/multi_logfile1";
  logfile_base2_ = tmp_dir_ + "/new-good/multi_logfile2";
  logfiles_ = MakeLogFiles(logfile_base1_, logfile_base2_);
  ASSERT_TRUE(pi1_logger.logger->RenameLogBase(logfile_base1_));
  ASSERT_TRUE(pi2_logger.logger->RenameLogBase(logfile_base2_));
  for (auto &file : logfiles_) {
    struct stat s;
    EXPECT_EQ(0, stat(file.c_str(), &s));
  }
}

// Test that renaming the file base dies.
TEST_P(MultinodeLoggerDeathTest, LoggerRenameFile) {
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(), BootTimestamp::epoch() + chrono::seconds(1000)});
  util::UnlinkRecursive(tmp_dir_ + "/renamefile");
  logfile_base1_ = tmp_dir_ + "/renamefile/multi_logfile1";
  logfile_base2_ = tmp_dir_ + "/renamefile/multi_logfile2";
  logfiles_ = MakeLogFiles(logfile_base1_, logfile_base2_);
  LoggerState pi1_logger = MakeLogger(pi1_);
  StartLogger(&pi1_logger);
  event_loop_factory_.RunFor(chrono::milliseconds(10000));
  logfile_base1_ = tmp_dir_ + "/new-renamefile/new_multi_logfile1";
  EXPECT_DEATH({ pi1_logger.logger->RenameLogBase(logfile_base1_); },
               "Rename of file base from");
}

// TODO(austin): We can write a test which recreates a logfile and confirms that
// we get it back.  That is the ultimate test.

// Tests that we properly recreate forwarded timestamps when replaying a log.
// This should be enough that we can then re-run the logger and get a valid log
// back.
TEST_P(MultinodeLoggerTest, RemoteReboot) {
  std::vector<std::string> actual_filenames;

  const UUID pi1_boot0 = UUID::Random();
  const UUID pi2_boot0 = UUID::Random();
  const UUID pi2_boot1 = UUID::Random();
  {
    CHECK_EQ(pi1_index_, 0u);
    CHECK_EQ(pi2_index_, 1u);

    time_converter_.set_boot_uuid(pi1_index_, 0, pi1_boot0);
    time_converter_.set_boot_uuid(pi2_index_, 0, pi2_boot0);
    time_converter_.set_boot_uuid(pi2_index_, 1, pi2_boot1);

    time_converter_.AddNextTimestamp(
        distributed_clock::epoch(),
        {BootTimestamp::epoch(), BootTimestamp::epoch()});
    const chrono::nanoseconds reboot_time = chrono::milliseconds(10100);
    time_converter_.AddNextTimestamp(
        distributed_clock::epoch() + reboot_time,
        {BootTimestamp::epoch() + reboot_time,
         BootTimestamp{
             .boot = 1,
             .time = monotonic_clock::epoch() + chrono::milliseconds(1323)}});
  }

  {
    LoggerState pi1_logger = MakeLogger(pi1_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));
    EXPECT_EQ(event_loop_factory_.GetNodeEventLoopFactory("pi1")->boot_uuid(),
              pi1_boot0);
    EXPECT_EQ(event_loop_factory_.GetNodeEventLoopFactory("pi2")->boot_uuid(),
              pi2_boot0);

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(10000));

    VLOG(1) << "Reboot now!";

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
    EXPECT_EQ(event_loop_factory_.GetNodeEventLoopFactory("pi1")->boot_uuid(),
              pi1_boot0);
    EXPECT_EQ(event_loop_factory_.GetNodeEventLoopFactory("pi2")->boot_uuid(),
              pi2_boot1);

    pi1_logger.AppendAllFilenames(&actual_filenames);
  }

  std::sort(actual_filenames.begin(), actual_filenames.end());
  std::sort(pi1_reboot_logfiles_.begin(), pi1_reboot_logfiles_.end());
  ASSERT_THAT(actual_filenames,
              ::testing::UnorderedElementsAreArray(pi1_reboot_logfiles_));

  // Confirm that our new oldest timestamps properly update as we reboot and
  // rotate.
  for (const std::string &file : pi1_reboot_logfiles_) {
    std::optional<SizePrefixedFlatbufferVector<LogFileHeader>> log_header =
        ReadHeader(file);
    CHECK(log_header);
    if (log_header->message().has_configuration()) {
      continue;
    }

    const monotonic_clock::time_point monotonic_start_time =
        monotonic_clock::time_point(
            chrono::nanoseconds(log_header->message().monotonic_start_time()));
    const UUID source_node_boot_uuid = UUID::FromString(
        log_header->message().source_node_boot_uuid()->string_view());

    if (log_header->message().node()->name()->string_view() != "pi1") {
      // The remote message channel should rotate later and have more parts.
      // This only is true on the log files with shared remote messages.
      //
      // TODO(austin): I'm not the most thrilled with this test pattern...  It
      // feels brittle in a different way.
      if (file.find("aos.message_bridge.RemoteMessage") == std::string::npos ||
          !shared()) {
        switch (log_header->message().parts_index()) {
          case 0:
            EXPECT_EQ(source_node_boot_uuid, pi2_boot0);
            EXPECT_EQ(monotonic_start_time, monotonic_clock::min_time);
            break;
          case 1:
            EXPECT_EQ(source_node_boot_uuid, pi2_boot0);
            ASSERT_EQ(monotonic_start_time,
                      monotonic_clock::epoch() + chrono::seconds(1));
            break;
          case 2:
            EXPECT_EQ(source_node_boot_uuid, pi2_boot1);
            EXPECT_EQ(monotonic_start_time, monotonic_clock::min_time) << file;
            break;
          case 3:
            EXPECT_EQ(source_node_boot_uuid, pi2_boot1);
            ASSERT_EQ(monotonic_start_time, monotonic_clock::epoch() +
                                                chrono::nanoseconds(2322999462))
                << " on " << file;
            break;
          default:
            FAIL();
            break;
        }
      } else {
        switch (log_header->message().parts_index()) {
          case 0:
          case 1:
            EXPECT_EQ(source_node_boot_uuid, pi2_boot0);
            EXPECT_EQ(monotonic_start_time, monotonic_clock::min_time);
            break;
          case 2:
            EXPECT_EQ(source_node_boot_uuid, pi2_boot0);
            ASSERT_EQ(monotonic_start_time,
                      monotonic_clock::epoch() + chrono::seconds(1));
            break;
          case 3:
          case 4:
            EXPECT_EQ(source_node_boot_uuid, pi2_boot1);
            EXPECT_EQ(monotonic_start_time, monotonic_clock::min_time) << file;
            break;
          case 5:
            EXPECT_EQ(source_node_boot_uuid, pi2_boot1);
            ASSERT_EQ(monotonic_start_time, monotonic_clock::epoch() +
                                                chrono::nanoseconds(2322999462))
                << " on " << file;
            break;
          default:
            FAIL();
            break;
        }
      }
      continue;
    }
    SCOPED_TRACE(file);
    SCOPED_TRACE(aos::FlatbufferToJson(
        *log_header, {.multi_line = true, .max_vector_size = 100}));
    ASSERT_TRUE(log_header->message().has_oldest_remote_monotonic_timestamps());
    ASSERT_EQ(
        log_header->message().oldest_remote_monotonic_timestamps()->size(), 2u);
    EXPECT_EQ(
        log_header->message().oldest_remote_monotonic_timestamps()->Get(0),
        monotonic_clock::max_time.time_since_epoch().count());
    ASSERT_TRUE(log_header->message().has_oldest_local_monotonic_timestamps());
    ASSERT_EQ(log_header->message().oldest_local_monotonic_timestamps()->size(),
              2u);
    EXPECT_EQ(log_header->message().oldest_local_monotonic_timestamps()->Get(0),
              monotonic_clock::max_time.time_since_epoch().count());
    ASSERT_TRUE(log_header->message()
                    .has_oldest_remote_unreliable_monotonic_timestamps());
    ASSERT_EQ(log_header->message()
                  .oldest_remote_unreliable_monotonic_timestamps()
                  ->size(),
              2u);
    EXPECT_EQ(log_header->message()
                  .oldest_remote_unreliable_monotonic_timestamps()
                  ->Get(0),
              monotonic_clock::max_time.time_since_epoch().count());
    ASSERT_TRUE(log_header->message()
                    .has_oldest_local_unreliable_monotonic_timestamps());
    ASSERT_EQ(log_header->message()
                  .oldest_local_unreliable_monotonic_timestamps()
                  ->size(),
              2u);
    EXPECT_EQ(log_header->message()
                  .oldest_local_unreliable_monotonic_timestamps()
                  ->Get(0),
              monotonic_clock::max_time.time_since_epoch().count());

    const monotonic_clock::time_point oldest_remote_monotonic_timestamps =
        monotonic_clock::time_point(chrono::nanoseconds(
            log_header->message().oldest_remote_monotonic_timestamps()->Get(
                1)));
    const monotonic_clock::time_point oldest_local_monotonic_timestamps =
        monotonic_clock::time_point(chrono::nanoseconds(
            log_header->message().oldest_local_monotonic_timestamps()->Get(1)));
    const monotonic_clock::time_point
        oldest_remote_unreliable_monotonic_timestamps =
            monotonic_clock::time_point(chrono::nanoseconds(
                log_header->message()
                    .oldest_remote_unreliable_monotonic_timestamps()
                    ->Get(1)));
    const monotonic_clock::time_point
        oldest_local_unreliable_monotonic_timestamps =
            monotonic_clock::time_point(chrono::nanoseconds(
                log_header->message()
                    .oldest_local_unreliable_monotonic_timestamps()
                    ->Get(1)));
    const monotonic_clock::time_point
        oldest_remote_reliable_monotonic_timestamps =
            monotonic_clock::time_point(chrono::nanoseconds(
                log_header->message()
                    .oldest_remote_reliable_monotonic_timestamps()
                    ->Get(1)));
    const monotonic_clock::time_point
        oldest_local_reliable_monotonic_timestamps =
            monotonic_clock::time_point(chrono::nanoseconds(
                log_header->message()
                    .oldest_local_reliable_monotonic_timestamps()
                    ->Get(1)));
    const monotonic_clock::time_point
        oldest_logger_remote_unreliable_monotonic_timestamps =
            monotonic_clock::time_point(chrono::nanoseconds(
                log_header->message()
                    .oldest_logger_remote_unreliable_monotonic_timestamps()
                    ->Get(0)));
    const monotonic_clock::time_point
        oldest_logger_local_unreliable_monotonic_timestamps =
            monotonic_clock::time_point(chrono::nanoseconds(
                log_header->message()
                    .oldest_logger_local_unreliable_monotonic_timestamps()
                    ->Get(0)));
    EXPECT_EQ(oldest_logger_remote_unreliable_monotonic_timestamps,
              monotonic_clock::max_time);
    EXPECT_EQ(oldest_logger_local_unreliable_monotonic_timestamps,
              monotonic_clock::max_time);
    switch (log_header->message().parts_index()) {
      case 0:
        EXPECT_EQ(oldest_remote_monotonic_timestamps,
                  monotonic_clock::max_time);
        EXPECT_EQ(oldest_local_monotonic_timestamps, monotonic_clock::max_time);
        EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::max_time);
        EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                  monotonic_clock::max_time);
        EXPECT_EQ(oldest_remote_reliable_monotonic_timestamps,
                  monotonic_clock::max_time);
        EXPECT_EQ(oldest_local_reliable_monotonic_timestamps,
                  monotonic_clock::max_time);
        break;
      case 1:
        EXPECT_EQ(oldest_remote_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(90200)));
        EXPECT_EQ(oldest_local_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(90350)));
        EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(90200)));
        EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(90350)));
        EXPECT_EQ(oldest_remote_reliable_monotonic_timestamps,
                  monotonic_clock::max_time);
        EXPECT_EQ(oldest_local_reliable_monotonic_timestamps,
                  monotonic_clock::max_time);
        break;
      case 2:
        EXPECT_EQ(oldest_remote_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(90200)))
            << file;
        EXPECT_EQ(oldest_local_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(90350)))
            << file;
        EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(90200)))
            << file;
        EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(90350)))
            << file;
        EXPECT_EQ(oldest_remote_reliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(100000)))
            << file;
        EXPECT_EQ(oldest_local_reliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(100150)))
            << file;
        break;
      case 3:
        EXPECT_EQ(oldest_remote_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::milliseconds(1323) +
                                              chrono::microseconds(200)));
        EXPECT_EQ(oldest_local_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(10100350)));
        EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::milliseconds(1323) +
                                              chrono::microseconds(200)));
        EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(10100350)));
        EXPECT_EQ(oldest_remote_reliable_monotonic_timestamps,
                  monotonic_clock::max_time)
            << file;
        EXPECT_EQ(oldest_local_reliable_monotonic_timestamps,
                  monotonic_clock::max_time)
            << file;
        break;
      case 4:
        EXPECT_EQ(oldest_remote_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::milliseconds(1323) +
                                              chrono::microseconds(200)));
        EXPECT_EQ(oldest_local_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(10100350)));
        EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::milliseconds(1323) +
                                              chrono::microseconds(200)));
        EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(10100350)));
        EXPECT_EQ(oldest_remote_reliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(1423000)))
            << file;
        EXPECT_EQ(oldest_local_reliable_monotonic_timestamps,
                  monotonic_clock::time_point(chrono::microseconds(10200150)))
            << file;
        break;
      default:
        FAIL();
        break;
    }
  }

  // Confirm that we refuse to replay logs with missing boot uuids.
  {
    LogReader reader(SortParts(pi1_reboot_logfiles_));

    SimulatedEventLoopFactory log_reader_factory(reader.configuration());
    log_reader_factory.set_send_delay(chrono::microseconds(0));

    // This sends out the fetched messages and advances time to the start of
    // the log file.
    reader.Register(&log_reader_factory);

    log_reader_factory.Run();

    reader.Deregister();
  }
}

// Tests that we can sort a log which only has timestamps from the remote
// because the local message_bridge_client failed to connect.
TEST_P(MultinodeLoggerTest, RemoteRebootOnlyTimestamps) {
  const UUID pi1_boot0 = UUID::Random();
  const UUID pi2_boot0 = UUID::Random();
  const UUID pi2_boot1 = UUID::Random();
  {
    CHECK_EQ(pi1_index_, 0u);
    CHECK_EQ(pi2_index_, 1u);

    time_converter_.set_boot_uuid(pi1_index_, 0, pi1_boot0);
    time_converter_.set_boot_uuid(pi2_index_, 0, pi2_boot0);
    time_converter_.set_boot_uuid(pi2_index_, 1, pi2_boot1);

    time_converter_.AddNextTimestamp(
        distributed_clock::epoch(),
        {BootTimestamp::epoch(), BootTimestamp::epoch()});
    const chrono::nanoseconds reboot_time = chrono::milliseconds(10100);
    time_converter_.AddNextTimestamp(
        distributed_clock::epoch() + reboot_time,
        {BootTimestamp::epoch() + reboot_time,
         BootTimestamp{
             .boot = 1,
             .time = monotonic_clock::epoch() + chrono::milliseconds(1323)}});
  }
  pi2_->Disconnect(pi1_->node());

  std::vector<std::string> filenames;
  {
    LoggerState pi1_logger = MakeLogger(pi1_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));
    EXPECT_EQ(event_loop_factory_.GetNodeEventLoopFactory("pi1")->boot_uuid(),
              pi1_boot0);
    EXPECT_EQ(event_loop_factory_.GetNodeEventLoopFactory("pi2")->boot_uuid(),
              pi2_boot0);

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(10000));

    VLOG(1) << "Reboot now!";

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
    EXPECT_EQ(event_loop_factory_.GetNodeEventLoopFactory("pi1")->boot_uuid(),
              pi1_boot0);
    EXPECT_EQ(event_loop_factory_.GetNodeEventLoopFactory("pi2")->boot_uuid(),
              pi2_boot1);
    pi1_logger.AppendAllFilenames(&filenames);
  }

  std::sort(filenames.begin(), filenames.end());

  // Confirm that our new oldest timestamps properly update as we reboot and
  // rotate.
  size_t timestamp_file_count = 0;
  for (const std::string &file : filenames) {
    std::optional<SizePrefixedFlatbufferVector<LogFileHeader>> log_header =
        ReadHeader(file);
    CHECK(log_header);

    if (log_header->message().has_configuration()) {
      continue;
    }

    const monotonic_clock::time_point monotonic_start_time =
        monotonic_clock::time_point(
            chrono::nanoseconds(log_header->message().monotonic_start_time()));
    const UUID source_node_boot_uuid = UUID::FromString(
        log_header->message().source_node_boot_uuid()->string_view());

    ASSERT_TRUE(log_header->message().has_oldest_remote_monotonic_timestamps());
    ASSERT_EQ(
        log_header->message().oldest_remote_monotonic_timestamps()->size(), 2u);
    ASSERT_TRUE(log_header->message().has_oldest_local_monotonic_timestamps());
    ASSERT_EQ(log_header->message().oldest_local_monotonic_timestamps()->size(),
              2u);
    ASSERT_TRUE(log_header->message()
                    .has_oldest_remote_unreliable_monotonic_timestamps());
    ASSERT_EQ(log_header->message()
                  .oldest_remote_unreliable_monotonic_timestamps()
                  ->size(),
              2u);
    ASSERT_TRUE(log_header->message()
                    .has_oldest_local_unreliable_monotonic_timestamps());
    ASSERT_EQ(log_header->message()
                  .oldest_local_unreliable_monotonic_timestamps()
                  ->size(),
              2u);
    ASSERT_TRUE(log_header->message()
                    .has_oldest_remote_reliable_monotonic_timestamps());
    ASSERT_EQ(log_header->message()
                  .oldest_remote_reliable_monotonic_timestamps()
                  ->size(),
              2u);
    ASSERT_TRUE(
        log_header->message().has_oldest_local_reliable_monotonic_timestamps());
    ASSERT_EQ(log_header->message()
                  .oldest_local_reliable_monotonic_timestamps()
                  ->size(),
              2u);

    ASSERT_TRUE(
        log_header->message()
            .has_oldest_logger_remote_unreliable_monotonic_timestamps());
    ASSERT_EQ(log_header->message()
                  .oldest_logger_remote_unreliable_monotonic_timestamps()
                  ->size(),
              2u);
    ASSERT_TRUE(log_header->message()
                    .has_oldest_logger_local_unreliable_monotonic_timestamps());
    ASSERT_EQ(log_header->message()
                  .oldest_logger_local_unreliable_monotonic_timestamps()
                  ->size(),
              2u);

    if (log_header->message().node()->name()->string_view() != "pi1") {
      ASSERT_TRUE(file.find("aos.message_bridge.RemoteMessage") !=
                  std::string::npos);

      const std::optional<SizePrefixedFlatbufferVector<MessageHeader>> msg =
          ReadNthMessage(file, 0);
      CHECK(msg);

      EXPECT_TRUE(msg->message().has_monotonic_sent_time());
      EXPECT_TRUE(msg->message().has_monotonic_remote_time());

      const monotonic_clock::time_point
          expected_oldest_local_monotonic_timestamps(
              chrono::nanoseconds(msg->message().monotonic_sent_time()));
      const monotonic_clock::time_point
          expected_oldest_remote_monotonic_timestamps(
              chrono::nanoseconds(msg->message().monotonic_remote_time()));
      const monotonic_clock::time_point
          expected_oldest_timestamp_monotonic_timestamps(
              chrono::nanoseconds(msg->message().monotonic_timestamp_time()));

      EXPECT_NE(expected_oldest_local_monotonic_timestamps,
                monotonic_clock::min_time);
      EXPECT_NE(expected_oldest_remote_monotonic_timestamps,
                monotonic_clock::min_time);
      EXPECT_NE(expected_oldest_timestamp_monotonic_timestamps,
                monotonic_clock::min_time);

      ++timestamp_file_count;
      // Since the log file is from the perspective of the other node,
      const monotonic_clock::time_point oldest_remote_monotonic_timestamps =
          monotonic_clock::time_point(chrono::nanoseconds(
              log_header->message().oldest_remote_monotonic_timestamps()->Get(
                  0)));
      const monotonic_clock::time_point oldest_local_monotonic_timestamps =
          monotonic_clock::time_point(chrono::nanoseconds(
              log_header->message().oldest_local_monotonic_timestamps()->Get(
                  0)));
      const monotonic_clock::time_point
          oldest_remote_unreliable_monotonic_timestamps =
              monotonic_clock::time_point(chrono::nanoseconds(
                  log_header->message()
                      .oldest_remote_unreliable_monotonic_timestamps()
                      ->Get(0)));
      const monotonic_clock::time_point
          oldest_local_unreliable_monotonic_timestamps =
              monotonic_clock::time_point(chrono::nanoseconds(
                  log_header->message()
                      .oldest_local_unreliable_monotonic_timestamps()
                      ->Get(0)));
      const monotonic_clock::time_point
          oldest_remote_reliable_monotonic_timestamps =
              monotonic_clock::time_point(chrono::nanoseconds(
                  log_header->message()
                      .oldest_remote_reliable_monotonic_timestamps()
                      ->Get(0)));
      const monotonic_clock::time_point
          oldest_local_reliable_monotonic_timestamps =
              monotonic_clock::time_point(chrono::nanoseconds(
                  log_header->message()
                      .oldest_local_reliable_monotonic_timestamps()
                      ->Get(0)));
      const monotonic_clock::time_point
          oldest_logger_remote_unreliable_monotonic_timestamps =
              monotonic_clock::time_point(chrono::nanoseconds(
                  log_header->message()
                      .oldest_logger_remote_unreliable_monotonic_timestamps()
                      ->Get(1)));
      const monotonic_clock::time_point
          oldest_logger_local_unreliable_monotonic_timestamps =
              monotonic_clock::time_point(chrono::nanoseconds(
                  log_header->message()
                      .oldest_logger_local_unreliable_monotonic_timestamps()
                      ->Get(1)));

      const Channel *channel =
          event_loop_factory_.configuration()->channels()->Get(
              msg->message().channel_index());
      const Connection *connection = configuration::ConnectionToNode(
          channel, configuration::GetNode(
                       event_loop_factory_.configuration(),
                       log_header->message().node()->name()->string_view()));

      const bool reliable = connection->time_to_live() == 0;

      SCOPED_TRACE(file);
      SCOPED_TRACE(aos::FlatbufferToJson(
          *log_header, {.multi_line = true, .max_vector_size = 100}));

      if (shared()) {
        // Confirm that the oldest timestamps match what we expect.  Based on
        // what we are doing, we know that the oldest time is the first
        // message's time.
        //
        // This makes the test robust to both the split and combined config
        // tests.
        switch (log_header->message().parts_index()) {
          case 0:
            EXPECT_EQ(oldest_remote_monotonic_timestamps,
                      expected_oldest_remote_monotonic_timestamps);
            EXPECT_EQ(oldest_local_monotonic_timestamps,
                      expected_oldest_local_monotonic_timestamps);
            EXPECT_EQ(oldest_logger_remote_unreliable_monotonic_timestamps,
                      expected_oldest_local_monotonic_timestamps)
                << file;
            EXPECT_EQ(oldest_logger_local_unreliable_monotonic_timestamps,
                      expected_oldest_timestamp_monotonic_timestamps)
                << file;

            if (reliable) {
              EXPECT_EQ(oldest_remote_reliable_monotonic_timestamps,
                        expected_oldest_remote_monotonic_timestamps);
              EXPECT_EQ(oldest_local_reliable_monotonic_timestamps,
                        expected_oldest_local_monotonic_timestamps);
              EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                        monotonic_clock::max_time);
              EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                        monotonic_clock::max_time);
            } else {
              EXPECT_EQ(oldest_remote_reliable_monotonic_timestamps,
                        monotonic_clock::max_time);
              EXPECT_EQ(oldest_local_reliable_monotonic_timestamps,
                        monotonic_clock::max_time);
              EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                        expected_oldest_remote_monotonic_timestamps);
              EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                        expected_oldest_local_monotonic_timestamps);
            }
            break;
          case 1:
            EXPECT_EQ(oldest_remote_monotonic_timestamps,
                      monotonic_clock::epoch() + chrono::nanoseconds(90000000));
            EXPECT_EQ(oldest_local_monotonic_timestamps,
                      monotonic_clock::epoch() + chrono::nanoseconds(90150000));
            EXPECT_EQ(oldest_logger_remote_unreliable_monotonic_timestamps,
                      monotonic_clock::epoch() + chrono::nanoseconds(90150000));
            EXPECT_EQ(oldest_logger_local_unreliable_monotonic_timestamps,
                      monotonic_clock::epoch() + chrono::nanoseconds(90250000));
            if (reliable) {
              EXPECT_EQ(oldest_remote_reliable_monotonic_timestamps,
                        expected_oldest_remote_monotonic_timestamps);
              EXPECT_EQ(oldest_local_reliable_monotonic_timestamps,
                        expected_oldest_local_monotonic_timestamps);
              EXPECT_EQ(
                  oldest_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::epoch() + chrono::nanoseconds(90000000));
              EXPECT_EQ(
                  oldest_local_unreliable_monotonic_timestamps,
                  monotonic_clock::epoch() + chrono::nanoseconds(90150000));
            } else {
              EXPECT_EQ(oldest_remote_reliable_monotonic_timestamps,
                        monotonic_clock::max_time);
              EXPECT_EQ(oldest_local_reliable_monotonic_timestamps,
                        monotonic_clock::max_time);
              EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                        expected_oldest_remote_monotonic_timestamps);
              EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                        expected_oldest_local_monotonic_timestamps);
            }
            break;
          case 2:
            EXPECT_EQ(
                oldest_remote_monotonic_timestamps,
                monotonic_clock::epoch() + chrono::nanoseconds(10000000000));
            EXPECT_EQ(
                oldest_local_monotonic_timestamps,
                monotonic_clock::epoch() + chrono::nanoseconds(1323100000));
            EXPECT_EQ(oldest_logger_remote_unreliable_monotonic_timestamps,
                      expected_oldest_local_monotonic_timestamps)
                << file;
            EXPECT_EQ(oldest_logger_local_unreliable_monotonic_timestamps,
                      expected_oldest_timestamp_monotonic_timestamps)
                << file;
            if (reliable) {
              EXPECT_EQ(oldest_remote_reliable_monotonic_timestamps,
                        expected_oldest_remote_monotonic_timestamps);
              EXPECT_EQ(oldest_local_reliable_monotonic_timestamps,
                        expected_oldest_local_monotonic_timestamps);
              EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                        monotonic_clock::max_time);
              EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                        monotonic_clock::max_time);
            } else {
              EXPECT_EQ(oldest_remote_reliable_monotonic_timestamps,
                        monotonic_clock::max_time);
              EXPECT_EQ(oldest_local_reliable_monotonic_timestamps,
                        monotonic_clock::max_time);
              EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                        expected_oldest_remote_monotonic_timestamps);
              EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                        expected_oldest_local_monotonic_timestamps);
            }
            break;

          case 3:
            EXPECT_EQ(
                oldest_remote_monotonic_timestamps,
                monotonic_clock::epoch() + chrono::nanoseconds(10000000000));
            EXPECT_EQ(
                oldest_local_monotonic_timestamps,
                monotonic_clock::epoch() + chrono::nanoseconds(1323100000));
            EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                      expected_oldest_remote_monotonic_timestamps);
            EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                      expected_oldest_local_monotonic_timestamps);
            EXPECT_EQ(
                oldest_logger_remote_unreliable_monotonic_timestamps,
                monotonic_clock::epoch() + chrono::nanoseconds(1323100000));
            EXPECT_EQ(
                oldest_logger_local_unreliable_monotonic_timestamps,
                monotonic_clock::epoch() + chrono::nanoseconds(10100200000));
            break;
          default:
            FAIL();
            break;
        }

        switch (log_header->message().parts_index()) {
          case 0:
            EXPECT_EQ(source_node_boot_uuid, pi2_boot0);
            EXPECT_EQ(monotonic_start_time, monotonic_clock::min_time);
            break;
          case 1:
            EXPECT_EQ(source_node_boot_uuid, pi2_boot0);
            EXPECT_EQ(monotonic_start_time, monotonic_clock::min_time);
            break;
          case 2:
            EXPECT_EQ(source_node_boot_uuid, pi2_boot1);
            EXPECT_EQ(monotonic_start_time, monotonic_clock::min_time);
            break;
          case 3:
            if (shared()) {
              EXPECT_EQ(source_node_boot_uuid, pi2_boot1);
              EXPECT_EQ(monotonic_start_time, monotonic_clock::min_time);
              break;
            }
            [[fallthrough]];
          default:
            FAIL();
            break;
        }
      } else {
        switch (log_header->message().parts_index()) {
          case 0:
            if (reliable) {
              EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                        monotonic_clock::max_time);
              EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                        monotonic_clock::max_time);
              EXPECT_EQ(
                  oldest_logger_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::epoch() + chrono::nanoseconds(100150000))
                  << file;
              EXPECT_EQ(
                  oldest_logger_local_unreliable_monotonic_timestamps,
                  monotonic_clock::epoch() + chrono::nanoseconds(100250000))
                  << file;
            } else {
              EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                        expected_oldest_remote_monotonic_timestamps);
              EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                        expected_oldest_local_monotonic_timestamps);
              EXPECT_EQ(
                  oldest_logger_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::epoch() + chrono::nanoseconds(90150000))
                  << file;
              EXPECT_EQ(
                  oldest_logger_local_unreliable_monotonic_timestamps,
                  monotonic_clock::epoch() + chrono::nanoseconds(90250000))
                  << file;
            }
            break;
          case 1:
            if (reliable) {
              EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                        monotonic_clock::max_time);
              EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                        monotonic_clock::max_time);
              EXPECT_EQ(
                  oldest_logger_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::epoch() + chrono::nanoseconds(1323100000));
              EXPECT_EQ(
                  oldest_logger_local_unreliable_monotonic_timestamps,
                  monotonic_clock::epoch() + chrono::nanoseconds(10100200000));
            } else {
              EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                        expected_oldest_remote_monotonic_timestamps);
              EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                        expected_oldest_local_monotonic_timestamps);
              EXPECT_EQ(
                  oldest_logger_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::epoch() + chrono::nanoseconds(1323150000));
              EXPECT_EQ(
                  oldest_logger_local_unreliable_monotonic_timestamps,
                  monotonic_clock::epoch() + chrono::nanoseconds(10100250000));
            }
            break;
          default:
            FAIL();
            break;
        }

        switch (log_header->message().parts_index()) {
          case 0:
            EXPECT_EQ(source_node_boot_uuid, pi2_boot0);
            EXPECT_EQ(monotonic_start_time, monotonic_clock::min_time);
            break;
          case 1:
            EXPECT_EQ(source_node_boot_uuid, pi2_boot1);
            EXPECT_EQ(monotonic_start_time, monotonic_clock::min_time);
            break;
          default:
            FAIL();
            break;
        }
      }

      continue;
    }
    EXPECT_EQ(
        log_header->message().oldest_remote_monotonic_timestamps()->Get(0),
        monotonic_clock::max_time.time_since_epoch().count());
    EXPECT_EQ(log_header->message().oldest_local_monotonic_timestamps()->Get(0),
              monotonic_clock::max_time.time_since_epoch().count());
    EXPECT_EQ(log_header->message()
                  .oldest_remote_unreliable_monotonic_timestamps()
                  ->Get(0),
              monotonic_clock::max_time.time_since_epoch().count());
    EXPECT_EQ(log_header->message()
                  .oldest_local_unreliable_monotonic_timestamps()
                  ->Get(0),
              monotonic_clock::max_time.time_since_epoch().count());

    const monotonic_clock::time_point oldest_remote_monotonic_timestamps =
        monotonic_clock::time_point(chrono::nanoseconds(
            log_header->message().oldest_remote_monotonic_timestamps()->Get(
                1)));
    const monotonic_clock::time_point oldest_local_monotonic_timestamps =
        monotonic_clock::time_point(chrono::nanoseconds(
            log_header->message().oldest_local_monotonic_timestamps()->Get(1)));
    const monotonic_clock::time_point
        oldest_remote_unreliable_monotonic_timestamps =
            monotonic_clock::time_point(chrono::nanoseconds(
                log_header->message()
                    .oldest_remote_unreliable_monotonic_timestamps()
                    ->Get(1)));
    const monotonic_clock::time_point
        oldest_local_unreliable_monotonic_timestamps =
            monotonic_clock::time_point(chrono::nanoseconds(
                log_header->message()
                    .oldest_local_unreliable_monotonic_timestamps()
                    ->Get(1)));
    switch (log_header->message().parts_index()) {
      case 0:
        EXPECT_EQ(oldest_remote_monotonic_timestamps,
                  monotonic_clock::max_time);
        EXPECT_EQ(oldest_local_monotonic_timestamps, monotonic_clock::max_time);
        EXPECT_EQ(oldest_remote_unreliable_monotonic_timestamps,
                  monotonic_clock::max_time);
        EXPECT_EQ(oldest_local_unreliable_monotonic_timestamps,
                  monotonic_clock::max_time);
        break;
      default:
        FAIL();
        break;
    }
  }

  if (shared()) {
    EXPECT_EQ(timestamp_file_count, 4u);
  } else {
    EXPECT_EQ(timestamp_file_count, 4u);
  }

  // Confirm that we can actually sort the resulting log and read it.
  {
    LogReader reader(SortParts(filenames));

    SimulatedEventLoopFactory log_reader_factory(reader.configuration());
    log_reader_factory.set_send_delay(chrono::microseconds(0));

    // This sends out the fetched messages and advances time to the start of
    // the log file.
    reader.Register(&log_reader_factory);

    log_reader_factory.Run();

    reader.Deregister();
  }
}

// Tests that we properly handle one direction of message_bridge being
// unavailable.
TEST_P(MultinodeLoggerTest, OneDirectionWithNegativeSlope) {
  pi1_->Disconnect(pi2_->node());
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(), BootTimestamp::epoch() + chrono::seconds(1000)});

  time_converter_.AddMonotonic(
      {chrono::milliseconds(10000),
       chrono::milliseconds(10000) - chrono::milliseconds(1)});
  {
    LoggerState pi1_logger = MakeLogger(pi1_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(10000));
  }

  // Confirm that we can parse the result.  LogReader has enough internal CHECKs
  // to confirm the right thing happened.
  ConfirmReadable(pi1_single_direction_logfiles_);
}

// Tests that we properly handle one direction of message_bridge being
// unavailable.
TEST_P(MultinodeLoggerTest, OneDirectionWithPositiveSlope) {
  pi1_->Disconnect(pi2_->node());
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(), BootTimestamp::epoch() + chrono::seconds(500)});

  time_converter_.AddMonotonic(
      {chrono::milliseconds(10000),
       chrono::milliseconds(10000) + chrono::milliseconds(1)});
  {
    LoggerState pi1_logger = MakeLogger(pi1_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(10000));
  }

  // Confirm that we can parse the result.  LogReader has enough internal CHECKs
  // to confirm the right thing happened.
  ConfirmReadable(pi1_single_direction_logfiles_);
}

// Tests that we explode if someone passes in a part file twice with a better
// error than an out of order error.
TEST_P(MultinodeLoggerTest, DuplicateLogFiles) {
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(), BootTimestamp::epoch() + chrono::seconds(1000)});
  {
    LoggerState pi1_logger = MakeLogger(pi1_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(10000));
  }

  std::vector<std::string> duplicates;
  for (const std::string &f : pi1_single_direction_logfiles_) {
    duplicates.emplace_back(f);
    duplicates.emplace_back(f);
  }
  EXPECT_DEATH({ SortParts(duplicates); }, "Found duplicate parts in");
}

// Tests that we explode if someone loses a part out of the middle of a log.
TEST_P(MultinodeLoggerTest, MissingPartsFromMiddle) {
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(), BootTimestamp::epoch() + chrono::seconds(1000)});
  {
    LoggerState pi1_logger = MakeLogger(pi1_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    aos::monotonic_clock::time_point last_rotation_time =
        pi1_logger.event_loop->monotonic_now();
    pi1_logger.logger->set_on_logged_period([&] {
      const auto now = pi1_logger.event_loop->monotonic_now();
      if (now > last_rotation_time + std::chrono::seconds(5)) {
        pi1_logger.logger->Rotate();
        last_rotation_time = now;
      }
    });

    event_loop_factory_.RunFor(chrono::milliseconds(10000));
  }

  std::vector<std::string> missing_parts;

  missing_parts.emplace_back(logfile_base1_ + "_pi1_data.part0" + Extension());
  missing_parts.emplace_back(logfile_base1_ + "_pi1_data.part2" + Extension());
  missing_parts.emplace_back(absl::StrCat(
      logfile_base1_, "_", std::get<0>(GetParam()).sha256, Extension()));

  EXPECT_DEATH({ SortParts(missing_parts); },
               "Broken log, missing part files between");
}

// Tests that we properly handle a dead node.  Do this by just disconnecting it
// and only using one nodes of logs.
TEST_P(MultinodeLoggerTest, DeadNode) {
  pi1_->Disconnect(pi2_->node());
  pi2_->Disconnect(pi1_->node());
  time_converter_.AddMonotonic(
      {BootTimestamp::epoch(), BootTimestamp::epoch() + chrono::seconds(1000)});
  {
    LoggerState pi1_logger = MakeLogger(pi1_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(10000));
  }

  // Confirm that we can parse the result.  LogReader has enough internal CHECKs
  // to confirm the right thing happened.
  ConfirmReadable(MakePi1DeadNodeLogfiles());
}

// Tests that we can relog with a different config.  This makes most sense when
// you are trying to edit a log and want to use channel renaming + the original
// config in the new log.
TEST_P(MultinodeLoggerTest, LogDifferentConfig) {
  time_converter_.StartEqual();
  {
    LoggerState pi1_logger = MakeLogger(pi1_);
    LoggerState pi2_logger = MakeLogger(pi2_);

    event_loop_factory_.RunFor(chrono::milliseconds(95));

    StartLogger(&pi1_logger);
    StartLogger(&pi2_logger);

    event_loop_factory_.RunFor(chrono::milliseconds(20000));
  }

  LogReader reader(SortParts(logfiles_));
  reader.RemapLoggedChannel<aos::examples::Ping>("/test", "/original");

  SimulatedEventLoopFactory log_reader_factory(reader.configuration());
  log_reader_factory.set_send_delay(chrono::microseconds(0));

  // This sends out the fetched messages and advances time to the start of the
  // log file.
  reader.Register(&log_reader_factory);

  const Node *pi1 =
      configuration::GetNode(log_reader_factory.configuration(), "pi1");
  const Node *pi2 =
      configuration::GetNode(log_reader_factory.configuration(), "pi2");

  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi1) << " pi1";
  LOG(INFO) << "Start time " << reader.monotonic_start_time(pi2) << " pi2";
  LOG(INFO) << "now pi1 "
            << log_reader_factory.GetNodeEventLoopFactory(pi1)->monotonic_now();
  LOG(INFO) << "now pi2 "
            << log_reader_factory.GetNodeEventLoopFactory(pi2)->monotonic_now();

  EXPECT_THAT(reader.LoggedNodes(),
              ::testing::ElementsAre(
                  configuration::GetNode(reader.logged_configuration(), pi1),
                  configuration::GetNode(reader.logged_configuration(), pi2)));

  reader.event_loop_factory()->set_send_delay(chrono::microseconds(0));

  // And confirm we can re-create a log again, while checking the contents.
  std::vector<std::string> log_files;
  {
    LoggerState pi1_logger =
        MakeLogger(log_reader_factory.GetNodeEventLoopFactory("pi1"),
                   &log_reader_factory, reader.logged_configuration());
    LoggerState pi2_logger =
        MakeLogger(log_reader_factory.GetNodeEventLoopFactory("pi2"),
                   &log_reader_factory, reader.logged_configuration());

    pi1_logger.StartLogger(tmp_dir_ + "/relogged1");
    pi2_logger.StartLogger(tmp_dir_ + "/relogged2");

    log_reader_factory.Run();

    for (auto &x : pi1_logger.log_namer->all_filenames()) {
      log_files.emplace_back(absl::StrCat(tmp_dir_, "/relogged1_", x));
    }
    for (auto &x : pi2_logger.log_namer->all_filenames()) {
      log_files.emplace_back(absl::StrCat(tmp_dir_, "/relogged2_", x));
    }
  }

  reader.Deregister();

  // And verify that we can run the LogReader over the relogged files without
  // hitting any fatal errors.
  {
    LogReader relogged_reader(SortParts(log_files));
    relogged_reader.Register();

    relogged_reader.event_loop_factory()->Run();
  }
}

// Tests that we properly replay a log where the start time for a node is before
// any data on the node.  This can happen if the logger starts before data is
// published.  While the scenario below is a bit convoluted, we have seen logs
// like this generated out in the wild.
TEST(MultinodeRebootLoggerTest, StartTimeBeforeData) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(ArtifactPath(
          "aos/events/logging/multinode_pingpong_split3_config.json"));
  message_bridge::TestingTimeConverter time_converter(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory event_loop_factory(&config.message());
  event_loop_factory.SetTimeConverter(&time_converter);
  NodeEventLoopFactory *const pi1 =
      event_loop_factory.GetNodeEventLoopFactory("pi1");
  const size_t pi1_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi1->node());
  NodeEventLoopFactory *const pi2 =
      event_loop_factory.GetNodeEventLoopFactory("pi2");
  const size_t pi2_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi2->node());
  NodeEventLoopFactory *const pi3 =
      event_loop_factory.GetNodeEventLoopFactory("pi3");
  const size_t pi3_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi3->node());

  const std::string kLogfile1_1 =
      aos::testing::TestTmpDir() + "/multi_logfile1/";
  const std::string kLogfile2_1 =
      aos::testing::TestTmpDir() + "/multi_logfile2.1/";
  const std::string kLogfile2_2 =
      aos::testing::TestTmpDir() + "/multi_logfile2.2/";
  const std::string kLogfile3_1 =
      aos::testing::TestTmpDir() + "/multi_logfile3/";
  util::UnlinkRecursive(kLogfile1_1);
  util::UnlinkRecursive(kLogfile2_1);
  util::UnlinkRecursive(kLogfile2_2);
  util::UnlinkRecursive(kLogfile3_1);
  const UUID pi1_boot0 = UUID::Random();
  const UUID pi2_boot0 = UUID::Random();
  const UUID pi2_boot1 = UUID::Random();
  const UUID pi3_boot0 = UUID::Random();
  {
    CHECK_EQ(pi1_index, 0u);
    CHECK_EQ(pi2_index, 1u);
    CHECK_EQ(pi3_index, 2u);

    time_converter.set_boot_uuid(pi1_index, 0, pi1_boot0);
    time_converter.set_boot_uuid(pi2_index, 0, pi2_boot0);
    time_converter.set_boot_uuid(pi2_index, 1, pi2_boot1);
    time_converter.set_boot_uuid(pi3_index, 0, pi3_boot0);

    time_converter.AddNextTimestamp(
        distributed_clock::epoch(),
        {BootTimestamp::epoch(), BootTimestamp::epoch(),
         BootTimestamp::epoch()});
    const chrono::nanoseconds reboot_time = chrono::milliseconds(20000);
    time_converter.AddNextTimestamp(
        distributed_clock::epoch() + reboot_time,
        {BootTimestamp::epoch() + reboot_time,
         BootTimestamp{
             .boot = 1,
             .time = monotonic_clock::epoch() + chrono::milliseconds(1323)},
         BootTimestamp::epoch() + reboot_time});
  }

  // Make everything perfectly quiet.
  event_loop_factory.SkipTimingReport();
  event_loop_factory.DisableStatistics();

  std::vector<std::string> filenames;
  {
    LoggerState pi1_logger = MakeLoggerState(
        pi1, &event_loop_factory, SupportedCompressionAlgorithms()[0]);
    LoggerState pi3_logger = MakeLoggerState(
        pi3, &event_loop_factory, SupportedCompressionAlgorithms()[0]);
    {
      // And now start the logger.
      LoggerState pi2_logger = MakeLoggerState(
          pi2, &event_loop_factory, SupportedCompressionAlgorithms()[0]);

      event_loop_factory.RunFor(chrono::milliseconds(1000));

      pi1_logger.StartLogger(kLogfile1_1);
      pi3_logger.StartLogger(kLogfile3_1);
      pi2_logger.StartLogger(kLogfile2_1);

      event_loop_factory.RunFor(chrono::milliseconds(10000));

      // Now that we've got a start time in the past, turn on data.
      event_loop_factory.EnableStatistics();
      std::unique_ptr<aos::EventLoop> ping_event_loop =
          pi1->MakeEventLoop("ping");
      Ping ping(ping_event_loop.get());

      pi2->AlwaysStart<Pong>("pong");

      event_loop_factory.RunFor(chrono::milliseconds(3000));

      pi2_logger.AppendAllFilenames(&filenames);

      // Stop logging on pi2 before rebooting and completely shut off all
      // messages on pi2.
      pi2->DisableStatistics();
      pi1->Disconnect(pi2->node());
      pi2->Disconnect(pi1->node());
    }
    event_loop_factory.RunFor(chrono::milliseconds(7000));
    // pi2 now reboots.
    {
      event_loop_factory.RunFor(chrono::milliseconds(1000));

      // Start logging again on pi2 after it is up.
      LoggerState pi2_logger = MakeLoggerState(
          pi2, &event_loop_factory, SupportedCompressionAlgorithms()[0]);
      pi2_logger.StartLogger(kLogfile2_2);

      event_loop_factory.RunFor(chrono::milliseconds(10000));
      // And, now that we have a start time in the log, turn data back on.
      pi2->EnableStatistics();
      pi1->Connect(pi2->node());
      pi2->Connect(pi1->node());

      pi2->AlwaysStart<Pong>("pong");
      std::unique_ptr<aos::EventLoop> ping_event_loop =
          pi1->MakeEventLoop("ping");
      Ping ping(ping_event_loop.get());

      event_loop_factory.RunFor(chrono::milliseconds(3000));

      pi2_logger.AppendAllFilenames(&filenames);
    }

    pi1_logger.AppendAllFilenames(&filenames);
    pi3_logger.AppendAllFilenames(&filenames);
  }

  // Confirm that we can parse the result.  LogReader has enough internal CHECKs
  // to confirm the right thing happened.
  const std::vector<LogFile> sorted_parts = SortParts(filenames);
  auto result = ConfirmReadable(filenames);
  EXPECT_THAT(result[0].first, ::testing::ElementsAre(realtime_clock::epoch() +
                                                      chrono::seconds(1)));
  EXPECT_THAT(result[0].second,
              ::testing::ElementsAre(realtime_clock::epoch() +
                                     chrono::microseconds(34990350)));

  EXPECT_THAT(result[1].first,
              ::testing::ElementsAre(
                  realtime_clock::epoch() + chrono::seconds(1),
                  realtime_clock::epoch() + chrono::microseconds(3323000)));
  EXPECT_THAT(result[1].second,
              ::testing::ElementsAre(
                  realtime_clock::epoch() + chrono::microseconds(13990200),
                  realtime_clock::epoch() + chrono::microseconds(16313200)));

  EXPECT_THAT(result[2].first, ::testing::ElementsAre(realtime_clock::epoch() +
                                                      chrono::seconds(1)));
  EXPECT_THAT(result[2].second,
              ::testing::ElementsAre(realtime_clock::epoch() +
                                     chrono::microseconds(34900150)));
}

// Tests that local data before remote data after reboot is properly replayed.
// We only trigger a reboot in the timestamp interpolation function when solving
// the timestamp problem when we actually have a point in the function.  This
// originally only happened when a point passes the noncausal filter.  At the
// start of time for the second boot, if we aren't careful, we will have
// messages which need to be published at times before the boot.  This happens
// when a local message is in the log before a forwarded message, so there is no
// point in the interpolation function.  This delays the reboot.  So, we need to
// recreate that situation and make sure it doesn't come back.
TEST(MultinodeRebootLoggerTest,
     LocalMessageBeforeRemoteBeforeStartAfterReboot) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(ArtifactPath(
          "aos/events/logging/multinode_pingpong_split3_config.json"));
  message_bridge::TestingTimeConverter time_converter(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory event_loop_factory(&config.message());
  event_loop_factory.SetTimeConverter(&time_converter);
  NodeEventLoopFactory *const pi1 =
      event_loop_factory.GetNodeEventLoopFactory("pi1");
  const size_t pi1_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi1->node());
  NodeEventLoopFactory *const pi2 =
      event_loop_factory.GetNodeEventLoopFactory("pi2");
  const size_t pi2_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi2->node());
  NodeEventLoopFactory *const pi3 =
      event_loop_factory.GetNodeEventLoopFactory("pi3");
  const size_t pi3_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi3->node());

  const std::string kLogfile1_1 =
      aos::testing::TestTmpDir() + "/multi_logfile1/";
  const std::string kLogfile2_1 =
      aos::testing::TestTmpDir() + "/multi_logfile2.1/";
  const std::string kLogfile2_2 =
      aos::testing::TestTmpDir() + "/multi_logfile2.2/";
  const std::string kLogfile3_1 =
      aos::testing::TestTmpDir() + "/multi_logfile3/";
  util::UnlinkRecursive(kLogfile1_1);
  util::UnlinkRecursive(kLogfile2_1);
  util::UnlinkRecursive(kLogfile2_2);
  util::UnlinkRecursive(kLogfile3_1);
  const UUID pi1_boot0 = UUID::Random();
  const UUID pi2_boot0 = UUID::Random();
  const UUID pi2_boot1 = UUID::Random();
  const UUID pi3_boot0 = UUID::Random();
  {
    CHECK_EQ(pi1_index, 0u);
    CHECK_EQ(pi2_index, 1u);
    CHECK_EQ(pi3_index, 2u);

    time_converter.set_boot_uuid(pi1_index, 0, pi1_boot0);
    time_converter.set_boot_uuid(pi2_index, 0, pi2_boot0);
    time_converter.set_boot_uuid(pi2_index, 1, pi2_boot1);
    time_converter.set_boot_uuid(pi3_index, 0, pi3_boot0);

    time_converter.AddNextTimestamp(
        distributed_clock::epoch(),
        {BootTimestamp::epoch(), BootTimestamp::epoch(),
         BootTimestamp::epoch()});
    const chrono::nanoseconds reboot_time = chrono::milliseconds(5000);
    time_converter.AddNextTimestamp(
        distributed_clock::epoch() + reboot_time,
        {BootTimestamp::epoch() + reboot_time,
         BootTimestamp{.boot = 1,
                       .time = monotonic_clock::epoch() + reboot_time +
                               chrono::seconds(100)},
         BootTimestamp::epoch() + reboot_time});
  }

  std::vector<std::string> filenames;
  {
    LoggerState pi1_logger = MakeLoggerState(
        pi1, &event_loop_factory, SupportedCompressionAlgorithms()[0]);
    LoggerState pi3_logger = MakeLoggerState(
        pi3, &event_loop_factory, SupportedCompressionAlgorithms()[0]);
    {
      // And now start the logger.
      LoggerState pi2_logger = MakeLoggerState(
          pi2, &event_loop_factory, SupportedCompressionAlgorithms()[0]);

      pi1_logger.StartLogger(kLogfile1_1);
      pi3_logger.StartLogger(kLogfile3_1);
      pi2_logger.StartLogger(kLogfile2_1);

      event_loop_factory.RunFor(chrono::milliseconds(1005));

      // Now that we've got a start time in the past, turn on data.
      std::unique_ptr<aos::EventLoop> ping_event_loop =
          pi1->MakeEventLoop("ping");
      Ping ping(ping_event_loop.get());

      pi2->AlwaysStart<Pong>("pong");

      event_loop_factory.RunFor(chrono::milliseconds(3000));

      pi2_logger.AppendAllFilenames(&filenames);

      // Disable any remote messages on pi2.
      pi1->Disconnect(pi2->node());
      pi2->Disconnect(pi1->node());
    }
    event_loop_factory.RunFor(chrono::milliseconds(995));
    // pi2 now reboots at 5 seconds.
    {
      event_loop_factory.RunFor(chrono::milliseconds(1000));

      // Make local stuff happen before we start logging and connect the remote.
      pi2->AlwaysStart<Pong>("pong");
      std::unique_ptr<aos::EventLoop> ping_event_loop =
          pi1->MakeEventLoop("ping");
      Ping ping(ping_event_loop.get());
      event_loop_factory.RunFor(chrono::milliseconds(1005));

      // Start logging again on pi2 after it is up.
      LoggerState pi2_logger = MakeLoggerState(
          pi2, &event_loop_factory, SupportedCompressionAlgorithms()[0]);
      pi2_logger.StartLogger(kLogfile2_2);

      // And allow remote messages now that we have some local ones.
      pi1->Connect(pi2->node());
      pi2->Connect(pi1->node());

      event_loop_factory.RunFor(chrono::milliseconds(1000));

      event_loop_factory.RunFor(chrono::milliseconds(3000));

      pi2_logger.AppendAllFilenames(&filenames);
    }

    pi1_logger.AppendAllFilenames(&filenames);
    pi3_logger.AppendAllFilenames(&filenames);
  }

  // Confirm that we can parse the result.  LogReader has enough internal CHECKs
  // to confirm the right thing happened.
  const std::vector<LogFile> sorted_parts = SortParts(filenames);
  auto result = ConfirmReadable(filenames);

  EXPECT_THAT(result[0].first, ::testing::ElementsAre(realtime_clock::epoch()));
  EXPECT_THAT(result[0].second,
              ::testing::ElementsAre(realtime_clock::epoch() +
                                     chrono::microseconds(11000350)));

  EXPECT_THAT(result[1].first,
              ::testing::ElementsAre(
                  realtime_clock::epoch(),
                  realtime_clock::epoch() + chrono::microseconds(107005000)));
  EXPECT_THAT(result[1].second,
              ::testing::ElementsAre(
                  realtime_clock::epoch() + chrono::microseconds(4000150),
                  realtime_clock::epoch() + chrono::microseconds(111000200)));

  EXPECT_THAT(result[2].first, ::testing::ElementsAre(realtime_clock::epoch()));
  EXPECT_THAT(result[2].second,
              ::testing::ElementsAre(realtime_clock::epoch() +
                                     chrono::microseconds(11000150)));

  auto start_stop_result = ConfirmReadable(
      filenames, realtime_clock::epoch() + chrono::milliseconds(2000),
      realtime_clock::epoch() + chrono::milliseconds(3000));

  EXPECT_THAT(
      start_stop_result[0].first,
      ::testing::ElementsAre(realtime_clock::epoch() + chrono::seconds(2)));
  EXPECT_THAT(
      start_stop_result[0].second,
      ::testing::ElementsAre(realtime_clock::epoch() + chrono::seconds(3)));
  EXPECT_THAT(
      start_stop_result[1].first,
      ::testing::ElementsAre(realtime_clock::epoch() + chrono::seconds(2)));
  EXPECT_THAT(
      start_stop_result[1].second,
      ::testing::ElementsAre(realtime_clock::epoch() + chrono::seconds(3)));
  EXPECT_THAT(
      start_stop_result[2].first,
      ::testing::ElementsAre(realtime_clock::epoch() + chrono::seconds(2)));
  EXPECT_THAT(
      start_stop_result[2].second,
      ::testing::ElementsAre(realtime_clock::epoch() + chrono::seconds(3)));
}

// Tests that setting the start and stop flags across a reboot works as
// expected.
TEST(MultinodeRebootLoggerTest, RebootStartStopTimes) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(ArtifactPath(
          "aos/events/logging/multinode_pingpong_split3_config.json"));
  message_bridge::TestingTimeConverter time_converter(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory event_loop_factory(&config.message());
  event_loop_factory.SetTimeConverter(&time_converter);
  NodeEventLoopFactory *const pi1 =
      event_loop_factory.GetNodeEventLoopFactory("pi1");
  const size_t pi1_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi1->node());
  NodeEventLoopFactory *const pi2 =
      event_loop_factory.GetNodeEventLoopFactory("pi2");
  const size_t pi2_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi2->node());
  NodeEventLoopFactory *const pi3 =
      event_loop_factory.GetNodeEventLoopFactory("pi3");
  const size_t pi3_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi3->node());

  const std::string kLogfile1_1 =
      aos::testing::TestTmpDir() + "/multi_logfile1/";
  const std::string kLogfile2_1 =
      aos::testing::TestTmpDir() + "/multi_logfile2.1/";
  const std::string kLogfile2_2 =
      aos::testing::TestTmpDir() + "/multi_logfile2.2/";
  const std::string kLogfile3_1 =
      aos::testing::TestTmpDir() + "/multi_logfile3/";
  util::UnlinkRecursive(kLogfile1_1);
  util::UnlinkRecursive(kLogfile2_1);
  util::UnlinkRecursive(kLogfile2_2);
  util::UnlinkRecursive(kLogfile3_1);
  {
    CHECK_EQ(pi1_index, 0u);
    CHECK_EQ(pi2_index, 1u);
    CHECK_EQ(pi3_index, 2u);

    time_converter.AddNextTimestamp(
        distributed_clock::epoch(),
        {BootTimestamp::epoch(), BootTimestamp::epoch(),
         BootTimestamp::epoch()});
    const chrono::nanoseconds reboot_time = chrono::milliseconds(5000);
    time_converter.AddNextTimestamp(
        distributed_clock::epoch() + reboot_time,
        {BootTimestamp::epoch() + reboot_time,
         BootTimestamp{.boot = 1,
                       .time = monotonic_clock::epoch() + reboot_time},
         BootTimestamp::epoch() + reboot_time});
  }

  std::vector<std::string> filenames;
  {
    LoggerState pi1_logger = MakeLoggerState(
        pi1, &event_loop_factory, SupportedCompressionAlgorithms()[0]);
    LoggerState pi3_logger = MakeLoggerState(
        pi3, &event_loop_factory, SupportedCompressionAlgorithms()[0]);
    {
      // And now start the logger.
      LoggerState pi2_logger = MakeLoggerState(
          pi2, &event_loop_factory, SupportedCompressionAlgorithms()[0]);

      pi1_logger.StartLogger(kLogfile1_1);
      pi3_logger.StartLogger(kLogfile3_1);
      pi2_logger.StartLogger(kLogfile2_1);

      event_loop_factory.RunFor(chrono::milliseconds(1005));

      // Now that we've got a start time in the past, turn on data.
      std::unique_ptr<aos::EventLoop> ping_event_loop =
          pi1->MakeEventLoop("ping");
      Ping ping(ping_event_loop.get());

      pi2->AlwaysStart<Pong>("pong");

      event_loop_factory.RunFor(chrono::milliseconds(3000));

      pi2_logger.AppendAllFilenames(&filenames);
    }
    event_loop_factory.RunFor(chrono::milliseconds(995));
    // pi2 now reboots at 5 seconds.
    {
      event_loop_factory.RunFor(chrono::milliseconds(1000));

      // Make local stuff happen before we start logging and connect the remote.
      pi2->AlwaysStart<Pong>("pong");
      std::unique_ptr<aos::EventLoop> ping_event_loop =
          pi1->MakeEventLoop("ping");
      Ping ping(ping_event_loop.get());
      event_loop_factory.RunFor(chrono::milliseconds(5));

      // Start logging again on pi2 after it is up.
      LoggerState pi2_logger = MakeLoggerState(
          pi2, &event_loop_factory, SupportedCompressionAlgorithms()[0]);
      pi2_logger.StartLogger(kLogfile2_2);

      event_loop_factory.RunFor(chrono::milliseconds(5000));

      pi2_logger.AppendAllFilenames(&filenames);
    }

    pi1_logger.AppendAllFilenames(&filenames);
    pi3_logger.AppendAllFilenames(&filenames);
  }

  const std::vector<LogFile> sorted_parts = SortParts(filenames);
  auto result = ConfirmReadable(filenames);

  EXPECT_THAT(result[0].first, ::testing::ElementsAre(realtime_clock::epoch()));
  EXPECT_THAT(result[0].second,
              ::testing::ElementsAre(realtime_clock::epoch() +
                                     chrono::microseconds(11000350)));

  EXPECT_THAT(result[1].first,
              ::testing::ElementsAre(
                  realtime_clock::epoch(),
                  realtime_clock::epoch() + chrono::microseconds(6005000)));
  EXPECT_THAT(result[1].second,
              ::testing::ElementsAre(
                  realtime_clock::epoch() + chrono::microseconds(4900150),
                  realtime_clock::epoch() + chrono::microseconds(11000200)));

  EXPECT_THAT(result[2].first, ::testing::ElementsAre(realtime_clock::epoch()));
  EXPECT_THAT(result[2].second,
              ::testing::ElementsAre(realtime_clock::epoch() +
                                     chrono::microseconds(11000150)));

  // Confirm we observed the correct start and stop times.  We should see the
  // reboot here.
  auto start_stop_result = ConfirmReadable(
      filenames, realtime_clock::epoch() + chrono::milliseconds(2000),
      realtime_clock::epoch() + chrono::milliseconds(8000));

  EXPECT_THAT(
      start_stop_result[0].first,
      ::testing::ElementsAre(realtime_clock::epoch() + chrono::seconds(2)));
  EXPECT_THAT(
      start_stop_result[0].second,
      ::testing::ElementsAre(realtime_clock::epoch() + chrono::seconds(8)));
  EXPECT_THAT(start_stop_result[1].first,
              ::testing::ElementsAre(
                  realtime_clock::epoch() + chrono::seconds(2),
                  realtime_clock::epoch() + chrono::microseconds(6005000)));
  EXPECT_THAT(start_stop_result[1].second,
              ::testing::ElementsAre(
                  realtime_clock::epoch() + chrono::microseconds(4900150),
                  realtime_clock::epoch() + chrono::seconds(8)));
  EXPECT_THAT(
      start_stop_result[2].first,
      ::testing::ElementsAre(realtime_clock::epoch() + chrono::seconds(2)));
  EXPECT_THAT(
      start_stop_result[2].second,
      ::testing::ElementsAre(realtime_clock::epoch() + chrono::seconds(8)));
}

// Tests that we properly handle one direction being down.
TEST(MissingDirectionTest, OneDirection) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(ArtifactPath(
          "aos/events/logging/multinode_pingpong_split4_config.json"));
  message_bridge::TestingTimeConverter time_converter(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory event_loop_factory(&config.message());
  event_loop_factory.SetTimeConverter(&time_converter);

  NodeEventLoopFactory *const pi1 =
      event_loop_factory.GetNodeEventLoopFactory("pi1");
  const size_t pi1_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi1->node());
  NodeEventLoopFactory *const pi2 =
      event_loop_factory.GetNodeEventLoopFactory("pi2");
  const size_t pi2_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi2->node());
  std::vector<std::string> filenames;

  {
    CHECK_EQ(pi1_index, 0u);
    CHECK_EQ(pi2_index, 1u);

    time_converter.AddNextTimestamp(
        distributed_clock::epoch(),
        {BootTimestamp::epoch(), BootTimestamp::epoch()});

    const chrono::nanoseconds reboot_time = chrono::milliseconds(5000);
    time_converter.AddNextTimestamp(
        distributed_clock::epoch() + reboot_time,
        {BootTimestamp{.boot = 1, .time = monotonic_clock::epoch()},
         BootTimestamp::epoch() + reboot_time});
  }

  const std::string kLogfile2_1 =
      aos::testing::TestTmpDir() + "/multi_logfile2.1/";
  const std::string kLogfile1_1 =
      aos::testing::TestTmpDir() + "/multi_logfile1.1/";
  util::UnlinkRecursive(kLogfile2_1);
  util::UnlinkRecursive(kLogfile1_1);

  pi2->Disconnect(pi1->node());

  pi1->AlwaysStart<Ping>("ping");
  pi2->AlwaysStart<Pong>("pong");

  {
    LoggerState pi2_logger = MakeLoggerState(
        pi2, &event_loop_factory, SupportedCompressionAlgorithms()[0]);

    event_loop_factory.RunFor(chrono::milliseconds(95));

    pi2_logger.StartLogger(kLogfile2_1);

    event_loop_factory.RunFor(chrono::milliseconds(6000));

    pi2->Connect(pi1->node());

    LoggerState pi1_logger = MakeLoggerState(
        pi1, &event_loop_factory, SupportedCompressionAlgorithms()[0]);
    pi1_logger.StartLogger(kLogfile1_1);

    event_loop_factory.RunFor(chrono::milliseconds(5000));
    pi1_logger.AppendAllFilenames(&filenames);
    pi2_logger.AppendAllFilenames(&filenames);
  }

  const std::vector<LogFile> sorted_parts = SortParts(filenames);
  ConfirmReadable(filenames);
}

// Tests that we properly handle only one direction ever existing after a
// reboot.
TEST(MissingDirectionTest, OneDirectionAfterReboot) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(ArtifactPath(
          "aos/events/logging/multinode_pingpong_split4_config.json"));
  message_bridge::TestingTimeConverter time_converter(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory event_loop_factory(&config.message());
  event_loop_factory.SetTimeConverter(&time_converter);

  NodeEventLoopFactory *const pi1 =
      event_loop_factory.GetNodeEventLoopFactory("pi1");
  const size_t pi1_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi1->node());
  NodeEventLoopFactory *const pi2 =
      event_loop_factory.GetNodeEventLoopFactory("pi2");
  const size_t pi2_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi2->node());
  std::vector<std::string> filenames;

  {
    CHECK_EQ(pi1_index, 0u);
    CHECK_EQ(pi2_index, 1u);

    time_converter.AddNextTimestamp(
        distributed_clock::epoch(),
        {BootTimestamp::epoch(), BootTimestamp::epoch()});

    const chrono::nanoseconds reboot_time = chrono::milliseconds(5000);
    time_converter.AddNextTimestamp(
        distributed_clock::epoch() + reboot_time,
        {BootTimestamp{.boot = 1, .time = monotonic_clock::epoch()},
         BootTimestamp::epoch() + reboot_time});
  }

  const std::string kLogfile2_1 =
      aos::testing::TestTmpDir() + "/multi_logfile2.1/";
  util::UnlinkRecursive(kLogfile2_1);

  pi1->AlwaysStart<Ping>("ping");

  // Pi1 sends to pi2.  Reboot pi1, but don't let pi2 connect to pi1.  This
  // makes it such that we will only get timestamps from pi1 -> pi2 on the
  // second boot.
  {
    LoggerState pi2_logger = MakeLoggerState(
        pi2, &event_loop_factory, SupportedCompressionAlgorithms()[0]);

    event_loop_factory.RunFor(chrono::milliseconds(95));

    pi2_logger.StartLogger(kLogfile2_1);

    event_loop_factory.RunFor(chrono::milliseconds(4000));

    pi2->Disconnect(pi1->node());

    event_loop_factory.RunFor(chrono::milliseconds(1000));
    pi1->AlwaysStart<Ping>("ping");

    event_loop_factory.RunFor(chrono::milliseconds(5000));
    pi2_logger.AppendAllFilenames(&filenames);
  }

  const std::vector<LogFile> sorted_parts = SortParts(filenames);
  ConfirmReadable(filenames);
}

// Tests that we properly handle only one direction ever existing after a reboot
// with only reliable data.
TEST(MissingDirectionTest, OneDirectionAfterRebootReliable) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(ArtifactPath(
          "aos/events/logging/multinode_pingpong_split4_reliable_config.json"));
  message_bridge::TestingTimeConverter time_converter(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory event_loop_factory(&config.message());
  event_loop_factory.SetTimeConverter(&time_converter);

  NodeEventLoopFactory *const pi1 =
      event_loop_factory.GetNodeEventLoopFactory("pi1");
  const size_t pi1_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi1->node());
  NodeEventLoopFactory *const pi2 =
      event_loop_factory.GetNodeEventLoopFactory("pi2");
  const size_t pi2_index = configuration::GetNodeIndex(
      event_loop_factory.configuration(), pi2->node());
  std::vector<std::string> filenames;

  {
    CHECK_EQ(pi1_index, 0u);
    CHECK_EQ(pi2_index, 1u);

    time_converter.AddNextTimestamp(
        distributed_clock::epoch(),
        {BootTimestamp::epoch(), BootTimestamp::epoch()});

    const chrono::nanoseconds reboot_time = chrono::milliseconds(5000);
    time_converter.AddNextTimestamp(
        distributed_clock::epoch() + reboot_time,
        {BootTimestamp{.boot = 1, .time = monotonic_clock::epoch()},
         BootTimestamp::epoch() + reboot_time});
  }

  const std::string kLogfile2_1 =
      aos::testing::TestTmpDir() + "/multi_logfile2.1/";
  util::UnlinkRecursive(kLogfile2_1);

  pi1->AlwaysStart<Ping>("ping");

  // Pi1 sends to pi2.  Reboot pi1, but don't let pi2 connect to pi1.  This
  // makes it such that we will only get timestamps from pi1 -> pi2 on the
  // second boot.
  {
    LoggerState pi2_logger = MakeLoggerState(
        pi2, &event_loop_factory, SupportedCompressionAlgorithms()[0]);

    event_loop_factory.RunFor(chrono::milliseconds(95));

    pi2_logger.StartLogger(kLogfile2_1);

    event_loop_factory.RunFor(chrono::milliseconds(4000));

    pi2->Disconnect(pi1->node());

    event_loop_factory.RunFor(chrono::milliseconds(1000));
    pi1->AlwaysStart<Ping>("ping");

    event_loop_factory.RunFor(chrono::milliseconds(5000));
    pi2_logger.AppendAllFilenames(&filenames);
  }

  const std::vector<LogFile> sorted_parts = SortParts(filenames);
  ConfirmReadable(filenames);
}

// Tests that we properly handle what used to be a time violation in one
// direction.  This can occur when one direction goes down after sending some
// data, but the other keeps working.  The down direction ends up resolving to a
// straight line in the noncausal filter, where the direction which is still up
// can cross that line.  Really, time progressed along just fine but we assumed
// that the offset was a line when it could have deviated by up to 1ms/second.
TEST_P(MultinodeLoggerTest, OneDirectionTimeDrift) {
  std::vector<std::string> filenames;

  CHECK_EQ(pi1_index_, 0u);
  CHECK_EQ(pi2_index_, 1u);

  time_converter_.AddNextTimestamp(
      distributed_clock::epoch(),
      {BootTimestamp::epoch(), BootTimestamp::epoch()});

  const chrono::nanoseconds before_disconnect_duration =
      time_converter_.AddMonotonic(
          {chrono::milliseconds(1000), chrono::milliseconds(1000)});

  const chrono::nanoseconds test_duration =
      time_converter_.AddMonotonic(
          {chrono::milliseconds(1000), chrono::milliseconds(1000)}) +
      time_converter_.AddMonotonic(
          {chrono::milliseconds(10000),
           chrono::milliseconds(10000) - chrono::milliseconds(5)}) +
      time_converter_.AddMonotonic(
          {chrono::milliseconds(10000),
           chrono::milliseconds(10000) + chrono::milliseconds(5)});

  const std::string kLogfile =
      aos::testing::TestTmpDir() + "/multi_logfile2.1/";
  util::UnlinkRecursive(kLogfile);

  {
    LoggerState pi2_logger = MakeLogger(pi2_);
    pi2_logger.StartLogger(kLogfile);
    event_loop_factory_.RunFor(before_disconnect_duration);

    pi2_->Disconnect(pi1_->node());

    event_loop_factory_.RunFor(test_duration);
    pi2_->Connect(pi1_->node());

    event_loop_factory_.RunFor(chrono::milliseconds(5000));
    pi2_logger.AppendAllFilenames(&filenames);
  }

  const std::vector<LogFile> sorted_parts = SortParts(filenames);
  ConfirmReadable(filenames);
}

// Tests that we can replay a logfile that has timestamps such that at least one
// node's epoch is at a positive distributed_clock (and thus will have to be
// booted after the other node(s)).
TEST_P(MultinodeLoggerTest, StartOneNodeBeforeOther) {
  std::vector<std::string> filenames;

  CHECK_EQ(pi1_index_, 0u);
  CHECK_EQ(pi2_index_, 1u);

  time_converter_.AddNextTimestamp(
      distributed_clock::epoch(),
      {BootTimestamp::epoch(), BootTimestamp::epoch()});

  const chrono::nanoseconds before_reboot_duration = chrono::milliseconds(1000);
  time_converter_.RebootAt(
      0, distributed_clock::time_point(before_reboot_duration));

  const chrono::nanoseconds test_duration = time_converter_.AddMonotonic(
      {chrono::milliseconds(10000), chrono::milliseconds(10000)});

  const std::string kLogfile =
      aos::testing::TestTmpDir() + "/multi_logfile2.1/";
  util::UnlinkRecursive(kLogfile);

  pi2_->Disconnect(pi1_->node());
  pi1_->Disconnect(pi2_->node());

  {
    LoggerState pi2_logger = MakeLogger(pi2_);

    pi2_logger.StartLogger(kLogfile);
    event_loop_factory_.RunFor(before_reboot_duration);

    pi2_->Connect(pi1_->node());
    pi1_->Connect(pi2_->node());

    event_loop_factory_.RunFor(test_duration);

    pi2_logger.AppendAllFilenames(&filenames);
  }

  const std::vector<LogFile> sorted_parts = SortParts(filenames);
  ConfirmReadable(filenames);

  {
    LogReader reader(sorted_parts);
    SimulatedEventLoopFactory replay_factory(reader.configuration());
    reader.RegisterWithoutStarting(&replay_factory);

    NodeEventLoopFactory *const replay_node =
        reader.event_loop_factory()->GetNodeEventLoopFactory("pi1");

    std::unique_ptr<EventLoop> test_event_loop =
        replay_node->MakeEventLoop("test_reader");
    replay_node->OnStartup([replay_node]() {
      // Check that we didn't boot until at least t=0.
      CHECK_LE(monotonic_clock::epoch(), replay_node->monotonic_now());
    });
    test_event_loop->OnRun([&test_event_loop]() {
      // Check that we didn't boot until at least t=0.
      EXPECT_LE(monotonic_clock::epoch(), test_event_loop->monotonic_now());
    });
    reader.event_loop_factory()->Run();
    reader.Deregister();
  }
}

// Tests that when we have a loop without all the logs at all points in time, we
// can sort it properly.
TEST(MultinodeLoggerLoopTest, Loop) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(ArtifactPath(
          "aos/events/logging/multinode_pingpong_triangle_split_config.json"));
  message_bridge::TestingTimeConverter time_converter(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory event_loop_factory(&config.message());
  event_loop_factory.SetTimeConverter(&time_converter);

  NodeEventLoopFactory *const pi1 =
      event_loop_factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *const pi2 =
      event_loop_factory.GetNodeEventLoopFactory("pi2");
  NodeEventLoopFactory *const pi3 =
      event_loop_factory.GetNodeEventLoopFactory("pi3");

  const std::string kLogfile1_1 =
      aos::testing::TestTmpDir() + "/multi_logfile1/";
  const std::string kLogfile2_1 =
      aos::testing::TestTmpDir() + "/multi_logfile2/";
  const std::string kLogfile3_1 =
      aos::testing::TestTmpDir() + "/multi_logfile3/";
  util::UnlinkRecursive(kLogfile1_1);
  util::UnlinkRecursive(kLogfile2_1);
  util::UnlinkRecursive(kLogfile3_1);

  {
    // Make pi1 boot before everything else.
    time_converter.AddNextTimestamp(
        distributed_clock::epoch(),
        {BootTimestamp::epoch(),
         BootTimestamp::epoch() - chrono::milliseconds(100),
         BootTimestamp::epoch() - chrono::milliseconds(300)});
  }

  // We want to setup a situation such that 2 of the 3 legs of the loop are very
  // confident about time being X, and the third leg is pulling the average off
  // to one side.
  //
  // It's easiest to visualize this in timestamp_plotter.

  std::vector<std::string> filenames;
  {
    // Have pi1 send out a reliable message at startup.  This sets up a long
    // forwarding time message at the start to bias time.
    std::unique_ptr<EventLoop> pi1_event_loop = pi1->MakeEventLoop("ping");
    {
      aos::Sender<examples::Ping> ping_sender =
          pi1_event_loop->MakeSender<examples::Ping>("/reliable");

      aos::Sender<examples::Ping>::Builder builder = ping_sender.MakeBuilder();
      examples::Ping::Builder ping_builder =
          builder.MakeBuilder<examples::Ping>();
      CHECK_EQ(builder.Send(ping_builder.Finish()), RawSender::Error::kOk);
    }

    // Wait a while so there's enough data to let the worst case be rather off.
    event_loop_factory.RunFor(chrono::seconds(1000));

    // Now start a receiving node first.  This sets up 2 tight bounds between 2
    // of the nodes.
    LoggerState pi2_logger = MakeLoggerState(
        pi2, &event_loop_factory, SupportedCompressionAlgorithms()[0]);
    pi2_logger.StartLogger(kLogfile2_1);

    event_loop_factory.RunFor(chrono::seconds(100));

    // And now start the third leg.
    LoggerState pi3_logger = MakeLoggerState(
        pi3, &event_loop_factory, SupportedCompressionAlgorithms()[0]);
    pi3_logger.StartLogger(kLogfile3_1);

    LoggerState pi1_logger = MakeLoggerState(
        pi1, &event_loop_factory, SupportedCompressionAlgorithms()[0]);
    pi1_logger.StartLogger(kLogfile1_1);

    event_loop_factory.RunFor(chrono::seconds(100));

    pi1_logger.AppendAllFilenames(&filenames);
    pi2_logger.AppendAllFilenames(&filenames);
    pi3_logger.AppendAllFilenames(&filenames);
  }

  // Make sure we can read this.
  const std::vector<LogFile> sorted_parts = SortParts(filenames);
  auto result = ConfirmReadable(filenames);
}

}  // namespace testing
}  // namespace logger
}  // namespace aos
