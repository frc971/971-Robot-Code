#ifndef AOS_EVENTS_LOGGING_MULTINODE_LOGGER_TEST_LIB_H
#define AOS_EVENTS_LOGGING_MULTINODE_LOGGER_TEST_LIB_H

#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "gmock/gmock.h"

#include "aos/events/event_loop.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/logging/snappy_encoder.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/testing_time_converter.h"
#include "aos/testing/path.h"
#include "aos/util/file.h"

#ifdef LZMA
#include "aos/events/logging/lzma_encoder.h"
#endif

namespace aos {
namespace logger {
namespace testing {

struct CompressionParams {
  std::string_view extension;
  std::function<std::unique_ptr<DataEncoder>(size_t max_message_size)>
      encoder_factory;
};

// Parameters to run all the tests with.
struct ConfigParams {
  // The config file to use.
  std::string config;
  // If true, the RemoteMessage channel should be shared between all the remote
  // channels.  If false, there will be 1 RemoteMessage channel per remote
  // channel.
  bool shared;
  // sha256 of the config.
  std::string_view sha256;
  // sha256 of the relogged config
  std::string_view relogged_sha256;
};

struct LoggerState {
  void StartLogger(std::string logfile_base);

  std::unique_ptr<MultiNodeFilesLogNamer> MakeLogNamer(
      std::string logfile_base);

  std::unique_ptr<EventLoop> event_loop;
  std::unique_ptr<Logger> logger;
  const Configuration *configuration;
  const Node *node;
  MultiNodeFilesLogNamer *log_namer;
  CompressionParams params;

  void AppendAllFilenames(std::vector<std::string> *filenames);

  ~LoggerState();
};

constexpr std::string_view kCombinedConfigSha1() {
  return "433bcf2bddfbbd2745a4e0c3c9dda2f9832bb61c5b311e3efdd357b9a19e1b76";
}
constexpr std::string_view kSplitConfigSha1() {
  return "6956d86e4eeda28d6857c3365f79a7fb0344c74de44bcb5ebe4d51398a4a26d5";
}
constexpr std::string_view kReloggedSplitConfigSha1() {
  return "db53e99234ecec2cde4d6b9f7b77c8f5150e0a58f6a441030eebfc1e76a2c89c";
}

LoggerState MakeLoggerState(NodeEventLoopFactory *node,
                            SimulatedEventLoopFactory *factory,
                            CompressionParams params,
                            const Configuration *configuration = nullptr);
std::vector<std::vector<std::string>> ToLogReaderVector(
    const std::vector<LogFile> &log_files);
std::vector<CompressionParams> SupportedCompressionAlgorithms();
std::ostream &operator<<(std::ostream &ostream,
                         const CompressionParams &params);
std::ostream &operator<<(std::ostream &ostream, const ConfigParams &params);
std::vector<std::pair<std::vector<realtime_clock::time_point>,
                      std::vector<realtime_clock::time_point>>>
ConfirmReadable(
    const std::vector<std::string> &files,
    realtime_clock::time_point start_time = realtime_clock::min_time,
    realtime_clock::time_point end_time = realtime_clock::max_time);
// Counts the number of messages on a channel.  Returns (channel name, channel
// type, count) for every message matching matcher()
std::vector<std::tuple<std::string, std::string, int>> CountChannelsMatching(
    std::shared_ptr<const aos::Configuration> config, std::string_view filename,
    std::function<bool(const UnpackedMessageHeader *)> matcher);
// Counts the number of messages (channel, count) for all data messages.
std::vector<std::tuple<std::string, std::string, int>> CountChannelsData(
    std::shared_ptr<const aos::Configuration> config,
    std::string_view filename);
// Counts the number of messages (channel, count) for all timestamp messages.
std::vector<std::tuple<std::string, std::string, int>> CountChannelsTimestamp(
    std::shared_ptr<const aos::Configuration> config,
    std::string_view filename);

class MultinodeLoggerTest : public ::testing::TestWithParam<
                                std::tuple<ConfigParams, CompressionParams>> {
 public:
  MultinodeLoggerTest();

  bool shared() const;

  std::vector<std::string> MakeLogFiles(std::string logfile_base1,
                                        std::string logfile_base2,
                                        size_t pi1_data_count = 3,
                                        size_t pi2_data_count = 3,
                                        bool relogged_config = false);

  std::vector<std::string> MakePi1RebootLogfiles();

  std::vector<std::string> MakePi1SingleDirectionLogfiles();

  std::vector<std::string> MakePi1DeadNodeLogfiles();

  std::vector<std::vector<std::string>> StructureLogFiles();

  std::string Extension();

  LoggerState MakeLogger(NodeEventLoopFactory *node,
                         SimulatedEventLoopFactory *factory = nullptr,
                         const Configuration *configuration = nullptr);

  void StartLogger(LoggerState *logger, std::string logfile_base = "");

  void VerifyParts(const std::vector<LogFile> &sorted_parts,
                   const std::vector<std::string> &corrupted_parts = {});

  void AddExtension(std::string_view extension);

  // Config and factory.
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  message_bridge::TestingTimeConverter time_converter_;
  SimulatedEventLoopFactory event_loop_factory_;

  NodeEventLoopFactory *const pi1_;
  const size_t pi1_index_;
  NodeEventLoopFactory *const pi2_;
  const size_t pi2_index_;

  std::string tmp_dir_;
  std::string logfile_base1_;
  std::string logfile_base2_;
  std::vector<std::string> pi1_reboot_logfiles_;
  std::vector<std::string> logfiles_;
  std::vector<std::string> pi1_single_direction_logfiles_;

  std::vector<std::vector<std::string>> structured_logfiles_;
};

typedef MultinodeLoggerTest MultinodeLoggerDeathTest;

}  // namespace testing
}  // namespace logger
}  // namespace aos

#endif  //  AOS_EVENTS_LOGGING_MULTINODE_LOGGER_TEST_LIB_H
