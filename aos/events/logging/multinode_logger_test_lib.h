#ifndef AOS_EVENTS_LOGGING_MULTINODE_LOGGER_TEST_LIB_H
#define AOS_EVENTS_LOGGING_MULTINODE_LOGGER_TEST_LIB_H

#include "absl/strings/str_format.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/logging/snappy_encoder.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/testing_time_converter.h"
#include "aos/testing/path.h"
#include "aos/util/file.h"
#include "glog/logging.h"
#include "gmock/gmock.h"

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

  std::unique_ptr<EventLoop> event_loop;
  std::unique_ptr<Logger> logger;
  const Configuration *configuration;
  const Node *node;
  MultiNodeLogNamer *log_namer;
  CompressionParams params;

  void AppendAllFilenames(std::vector<std::string> *filenames);

  ~LoggerState();
};

constexpr std::string_view kCombinedConfigSha1() {
  return "c8cd3762e42a4e19b2155f63ccec97d1627a2fbd34d3da3ea6541128ca22b899";
}
constexpr std::string_view kSplitConfigSha1() {
  return "0ee6360b3e82a46f3f8b241661934abac53957d494a81ed1938899c220334954";
}
constexpr std::string_view kReloggedSplitConfigSha1() {
  return "cc31e1a644dd7bf65d72247aea3e09b3474753e01921f3b6272f8233f288a16b";
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
