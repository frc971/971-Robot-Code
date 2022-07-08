#include "aos/events/logging/multinode_logger_test_lib.h"

#include "aos/events/event_loop.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/testing/tmpdir.h"

namespace aos {
namespace logger {
namespace testing {

using aos::testing::ArtifactPath;

LoggerState MakeLoggerState(NodeEventLoopFactory *node,
                            SimulatedEventLoopFactory *factory,
                            CompressionParams params,
                            const Configuration *configuration) {
  if (configuration == nullptr) {
    configuration = factory->configuration();
  }
  return {node->MakeEventLoop("logger"),
          {},
          configuration,
          configuration::GetNode(configuration, node->node()),
          nullptr,
          params};
}

void LoggerState::StartLogger(std::string logfile_base) {
  CHECK(!logfile_base.empty());

  logger = std::make_unique<Logger>(event_loop.get(), configuration);
  logger->set_polling_period(std::chrono::milliseconds(100));
  logger->set_name(
      absl::StrCat("name_prefix_", event_loop->node()->name()->str()));
  logger->set_logger_sha1(
      absl::StrCat("logger_sha1_", event_loop->node()->name()->str()));
  logger->set_logger_version(
      absl::StrCat("logger_version_", event_loop->node()->name()->str()));
  event_loop->OnRun([this, logfile_base]() {
    std::unique_ptr<MultiNodeFilesLogNamer> namer =
        std::make_unique<MultiNodeFilesLogNamer>(logfile_base, configuration,
                                            event_loop.get(), node);
    namer->set_extension(params.extension);
    namer->set_encoder_factory(params.encoder_factory);
    log_namer = namer.get();

    logger->StartLogging(std::move(namer));
  });
}

void LoggerState::AppendAllFilenames(std::vector<std::string> *filenames) {
  for (const std::string &file : log_namer->all_filenames()) {
    const std::string_view separator =
        log_namer->base_name().back() == '/' ? "" : "_";
    filenames->emplace_back(
        absl::StrCat(log_namer->base_name(), separator, file));
  }
}

LoggerState::~LoggerState() {
  if (logger) {
    std::vector<std::string> filenames;
    AppendAllFilenames(&filenames);
    std::sort(filenames.begin(), filenames.end());
    for (const std::string &file : filenames) {
      LOG(INFO) << "Wrote to " << file;
      auto x = ReadHeader(file);
      if (x) {
        VLOG(1) << aos::FlatbufferToJson(x.value());
      }
    }
  }
}

MultinodeLoggerTest::MultinodeLoggerTest()
    : config_(aos::configuration::ReadConfig(ArtifactPath(absl::StrCat(
          "aos/events/logging/", std::get<0>(GetParam()).config)))),
      time_converter_(configuration::NodesCount(&config_.message())),
      event_loop_factory_(&config_.message()),
      pi1_(event_loop_factory_.GetNodeEventLoopFactory("pi1")),
      pi1_index_(configuration::GetNodeIndex(
          event_loop_factory_.configuration(), pi1_->node())),
      pi2_(event_loop_factory_.GetNodeEventLoopFactory("pi2")),
      pi2_index_(configuration::GetNodeIndex(
          event_loop_factory_.configuration(), pi2_->node())),
      tmp_dir_(aos::testing::TestTmpDir()),
      logfile_base1_(tmp_dir_ + "/multi_logfile1"),
      logfile_base2_(tmp_dir_ + "/multi_logfile2"),
      pi1_reboot_logfiles_(MakePi1RebootLogfiles()),
      logfiles_(MakeLogFiles(logfile_base1_, logfile_base2_)),
      pi1_single_direction_logfiles_(MakePi1SingleDirectionLogfiles()),
      structured_logfiles_(StructureLogFiles()) {
  LOG(INFO) << "Config " << std::get<0>(GetParam()).config;
  event_loop_factory_.SetTimeConverter(&time_converter_);

  // Go through and remove the logfiles if they already exist.
  for (const auto &file : logfiles_) {
    unlink(file.c_str());
    unlink((file + ".xz").c_str());
  }

  for (const auto &file : MakeLogFiles(tmp_dir_ + "/relogged1",
                                       tmp_dir_ + "/relogged2", 3, 3, true)) {
    unlink(file.c_str());
  }

  for (const auto &file : pi1_reboot_logfiles_) {
    unlink(file.c_str());
  }

  LOG(INFO) << "Logging data to " << logfiles_[0] << ", " << logfiles_[1]
            << " and " << logfiles_[2];

  pi1_->OnStartup([this]() {
    pi1_->AlwaysStart<Ping>("ping");
    pi1_->AlwaysStart<Ping>("ping_local", "/aos");
  });
  pi2_->OnStartup([this]() {
    pi2_->AlwaysStart<Pong>("pong");
    pi2_->AlwaysStart<Ping>("ping_local", "/aos");
  });
}

bool MultinodeLoggerTest::shared() const {
  return std::get<0>(GetParam()).shared;
}

std::vector<std::string> MultinodeLoggerTest::MakeLogFiles(
    std::string logfile_base1, std::string logfile_base2, size_t pi1_data_count,
    size_t pi2_data_count, bool relogged_config) {
  std::string_view sha256 = relogged_config
                                ? std::get<0>(GetParam()).relogged_sha256
                                : std::get<0>(GetParam()).sha256;
  std::vector<std::string> result;
  result.emplace_back(absl::StrCat(logfile_base1, "_", sha256, Extension()));
  result.emplace_back(absl::StrCat(logfile_base2, "_", sha256, Extension()));
  for (size_t i = 0; i < pi1_data_count; ++i) {
    result.emplace_back(
        absl::StrCat(logfile_base1, "_pi1_data.part", i, Extension()));
  }
  result.emplace_back(logfile_base1 + "_pi2_data/test/aos.examples.Pong.part0" +
                      Extension());
  result.emplace_back(logfile_base1 + "_pi2_data/test/aos.examples.Pong.part1" +
                      Extension());
  for (size_t i = 0; i < pi2_data_count; ++i) {
    result.emplace_back(
        absl::StrCat(logfile_base2, "_pi2_data.part", i, Extension()));
  }
  result.emplace_back(logfile_base2 +
                      "_pi1_data/pi1/aos/aos.message_bridge.Timestamp.part0" +
                      Extension());
  result.emplace_back(logfile_base2 +
                      "_pi1_data/pi1/aos/aos.message_bridge.Timestamp.part1" +
                      Extension());
  result.emplace_back(logfile_base1 +
                      "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part0" +
                      Extension());
  result.emplace_back(logfile_base1 +
                      "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part1" +
                      Extension());
  if (shared()) {
    result.emplace_back(logfile_base1 +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/"
                        "aos.message_bridge.RemoteMessage.part0" +
                        Extension());
    result.emplace_back(logfile_base1 +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/"
                        "aos.message_bridge.RemoteMessage.part1" +
                        Extension());
    result.emplace_back(logfile_base1 +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/"
                        "aos.message_bridge.RemoteMessage.part2" +
                        Extension());
    result.emplace_back(logfile_base2 +
                        "_timestamps/pi2/aos/remote_timestamps/pi1/"
                        "aos.message_bridge.RemoteMessage.part0" +
                        Extension());
    result.emplace_back(logfile_base2 +
                        "_timestamps/pi2/aos/remote_timestamps/pi1/"
                        "aos.message_bridge.RemoteMessage.part1" +
                        Extension());
  } else {
    result.emplace_back(logfile_base1 +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                        "aos-message_bridge-Timestamp/"
                        "aos.message_bridge.RemoteMessage.part0" +
                        Extension());
    result.emplace_back(logfile_base1 +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                        "aos-message_bridge-Timestamp/"
                        "aos.message_bridge.RemoteMessage.part1" +
                        Extension());
    result.emplace_back(logfile_base2 +
                        "_timestamps/pi2/aos/remote_timestamps/pi1/pi2/aos/"
                        "aos-message_bridge-Timestamp/"
                        "aos.message_bridge.RemoteMessage.part0" +
                        Extension());
    result.emplace_back(logfile_base2 +
                        "_timestamps/pi2/aos/remote_timestamps/pi1/pi2/aos/"
                        "aos-message_bridge-Timestamp/"
                        "aos.message_bridge.RemoteMessage.part1" +
                        Extension());
    result.emplace_back(logfile_base1 +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/test/"
                        "aos-examples-Ping/"
                        "aos.message_bridge.RemoteMessage.part0" +
                        Extension());
    result.emplace_back(logfile_base1 +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/test/"
                        "aos-examples-Ping/"
                        "aos.message_bridge.RemoteMessage.part1" +
                        Extension());
  }

  return result;
}

std::vector<std::string> MultinodeLoggerTest::MakePi1RebootLogfiles() {
  std::vector<std::string> result;
  result.emplace_back(logfile_base1_ + "_pi1_data.part0" + Extension());
  result.emplace_back(logfile_base1_ + "_pi1_data.part1" + Extension());
  result.emplace_back(logfile_base1_ + "_pi1_data.part2" + Extension());
  result.emplace_back(logfile_base1_ + "_pi1_data.part3" + Extension());
  result.emplace_back(logfile_base1_ + "_pi1_data.part4" + Extension());
  result.emplace_back(logfile_base1_ +
                      "_pi2_data/test/aos.examples.Pong.part0" + Extension());
  result.emplace_back(logfile_base1_ +
                      "_pi2_data/test/aos.examples.Pong.part1" + Extension());
  result.emplace_back(logfile_base1_ +
                      "_pi2_data/test/aos.examples.Pong.part2" + Extension());
  result.emplace_back(logfile_base1_ +
                      "_pi2_data/test/aos.examples.Pong.part3" + Extension());
  result.emplace_back(logfile_base1_ +
                      "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part0" +
                      Extension());
  result.emplace_back(logfile_base1_ +
                      "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part1" +
                      Extension());
  result.emplace_back(logfile_base1_ +
                      "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part2" +
                      Extension());
  result.emplace_back(logfile_base1_ +
                      "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part3" +
                      Extension());
  result.emplace_back(absl::StrCat(
      logfile_base1_, "_", std::get<0>(GetParam()).sha256, Extension()));
  if (shared()) {
    for (size_t i = 0; i < 6; ++i) {
      result.emplace_back(
          absl::StrCat(logfile_base1_,
                       "_timestamps/pi1/aos/remote_timestamps/pi2/"
                       "aos.message_bridge.RemoteMessage.part",
                       i, Extension()));
    }
  } else {
    result.emplace_back(logfile_base1_ +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                        "aos-message_bridge-Timestamp/"
                        "aos.message_bridge.RemoteMessage.part0" +
                        Extension());
    result.emplace_back(logfile_base1_ +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                        "aos-message_bridge-Timestamp/"
                        "aos.message_bridge.RemoteMessage.part1" +
                        Extension());
    result.emplace_back(logfile_base1_ +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                        "aos-message_bridge-Timestamp/"
                        "aos.message_bridge.RemoteMessage.part2" +
                        Extension());
    result.emplace_back(logfile_base1_ +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                        "aos-message_bridge-Timestamp/"
                        "aos.message_bridge.RemoteMessage.part3" +
                        Extension());

    result.emplace_back(logfile_base1_ +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/test/"
                        "aos-examples-Ping/"
                        "aos.message_bridge.RemoteMessage.part0" +
                        Extension());
    result.emplace_back(logfile_base1_ +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/test/"
                        "aos-examples-Ping/"
                        "aos.message_bridge.RemoteMessage.part1" +
                        Extension());
    result.emplace_back(logfile_base1_ +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/test/"
                        "aos-examples-Ping/"
                        "aos.message_bridge.RemoteMessage.part2" +
                        Extension());
    result.emplace_back(logfile_base1_ +
                        "_timestamps/pi1/aos/remote_timestamps/pi2/test/"
                        "aos-examples-Ping/"
                        "aos.message_bridge.RemoteMessage.part3" +
                        Extension());
  }
  return result;
}

std::vector<std::string> MultinodeLoggerTest::MakePi1SingleDirectionLogfiles() {
  std::vector<std::string> result;
  result.emplace_back(logfile_base1_ + "_pi1_data.part0" + Extension());
  result.emplace_back(logfile_base1_ + "_pi1_data.part1" + Extension());
  result.emplace_back(logfile_base1_ +
                      "_pi2_data/pi2/aos/aos.message_bridge.Timestamp.part0" +
                      Extension());
  result.emplace_back(absl::StrCat(
      logfile_base1_, "_", std::get<0>(GetParam()).sha256, Extension()));
  return result;
}

std::vector<std::string> MultinodeLoggerTest::MakePi1DeadNodeLogfiles() {
  std::vector<std::string> result;
  result.emplace_back(logfile_base1_ + "_pi1_data.part0" + Extension());
  result.emplace_back(absl::StrCat(
      logfile_base1_, "_", std::get<0>(GetParam()).sha256, Extension()));
  return result;
}

std::vector<std::vector<std::string>> MultinodeLoggerTest::StructureLogFiles() {
  std::vector<std::vector<std::string>> result{
      std::vector<std::string>{logfiles_[2], logfiles_[3], logfiles_[4]},
      std::vector<std::string>{logfiles_[5], logfiles_[6]},
      std::vector<std::string>{logfiles_[7], logfiles_[8], logfiles_[9]},
      std::vector<std::string>{logfiles_[10], logfiles_[11]},
      std::vector<std::string>{logfiles_[12], logfiles_[13]}};

  if (shared()) {
    result.emplace_back(
        std::vector<std::string>{logfiles_[14], logfiles_[15], logfiles_[16]});
    result.emplace_back(std::vector<std::string>{logfiles_[17], logfiles_[18]});
  } else {
    result.emplace_back(std::vector<std::string>{logfiles_[14], logfiles_[15]});
    result.emplace_back(std::vector<std::string>{logfiles_[16], logfiles_[17]});
    result.emplace_back(std::vector<std::string>{logfiles_[18], logfiles_[19]});
  }

  return result;
}

std::string MultinodeLoggerTest::Extension() {
  return absl::StrCat(".bfbs", std::get<1>(GetParam()).extension);
}

LoggerState MultinodeLoggerTest::MakeLogger(
    NodeEventLoopFactory *node, SimulatedEventLoopFactory *factory,
    const Configuration *configuration) {
  if (factory == nullptr) {
    factory = &event_loop_factory_;
  }
  return MakeLoggerState(node, factory, std::get<1>(GetParam()), configuration);
}

void MultinodeLoggerTest::StartLogger(LoggerState *logger,
                                      std::string logfile_base) {
  if (logfile_base.empty()) {
    if (logger->event_loop->node()->name()->string_view() == "pi1") {
      logfile_base = logfile_base1_;
    } else {
      logfile_base = logfile_base2_;
    }
  }
  logger->StartLogger(logfile_base);
}

void MultinodeLoggerTest::VerifyParts(
    const std::vector<LogFile> &sorted_parts,
    const std::vector<std::string> &corrupted_parts) {
  EXPECT_EQ(sorted_parts.size(), 2u);

  // Count up the number of UUIDs and make sure they are what we expect as a
  // sanity check.
  std::set<std::string> log_event_uuids;
  std::set<std::string> parts_uuids;
  std::set<std::string> both_uuids;

  size_t missing_rt_count = 0;

  std::vector<std::string> logger_nodes;
  for (const LogFile &log_file : sorted_parts) {
    EXPECT_FALSE(log_file.log_event_uuid.empty());
    log_event_uuids.insert(log_file.log_event_uuid);
    logger_nodes.emplace_back(log_file.logger_node);
    both_uuids.insert(log_file.log_event_uuid);
    EXPECT_TRUE(log_file.config);
    EXPECT_EQ(log_file.name,
              absl::StrCat("name_prefix_", log_file.logger_node));
    EXPECT_EQ(log_file.logger_sha1,
              absl::StrCat("logger_sha1_", log_file.logger_node));
    EXPECT_EQ(log_file.logger_version,
              absl::StrCat("logger_version_", log_file.logger_node));

    for (const LogParts &part : log_file.parts) {
      EXPECT_NE(part.monotonic_start_time, aos::monotonic_clock::min_time)
          << ": " << part;
      missing_rt_count +=
          part.realtime_start_time == aos::realtime_clock::min_time;

      EXPECT_TRUE(log_event_uuids.find(part.log_event_uuid) !=
                  log_event_uuids.end());
      EXPECT_NE(part.node, "");
      EXPECT_TRUE(log_file.config);
      parts_uuids.insert(part.parts_uuid);
      both_uuids.insert(part.parts_uuid);
    }
  }

  // We won't have RT timestamps for 5 or 6 log files.  We don't log the RT
  // start time on remote nodes because we don't know it and would be
  // guessing.  And the log reader can actually do a better job.  The number
  // depends on if we have the remote timestamps split across 2 files, or just
  // across 1, depending on if we are using a split or combined timestamp
  // channel config.
  EXPECT_EQ(missing_rt_count, shared() ? 5u : 6u);

  EXPECT_EQ(log_event_uuids.size(), 2u);
  EXPECT_EQ(parts_uuids.size(), ToLogReaderVector(sorted_parts).size());
  EXPECT_EQ(log_event_uuids.size() + parts_uuids.size(), both_uuids.size());

  // Test that each list of parts is in order.  Don't worry about the ordering
  // between part file lists though.
  // (inner vectors all need to be in order, but outer one doesn't matter).
  ASSERT_THAT(ToLogReaderVector(sorted_parts),
              ::testing::UnorderedElementsAreArray(structured_logfiles_));

  EXPECT_THAT(logger_nodes, ::testing::UnorderedElementsAre("pi1", "pi2"));

  EXPECT_NE(sorted_parts[0].realtime_start_time, aos::realtime_clock::min_time);
  EXPECT_NE(sorted_parts[1].realtime_start_time, aos::realtime_clock::min_time);

  EXPECT_NE(sorted_parts[0].monotonic_start_time,
            aos::monotonic_clock::min_time);
  EXPECT_NE(sorted_parts[1].monotonic_start_time,
            aos::monotonic_clock::min_time);

  EXPECT_THAT(sorted_parts[0].corrupted, ::testing::Eq(corrupted_parts));
  EXPECT_THAT(sorted_parts[1].corrupted, ::testing::Eq(corrupted_parts));
}

void MultinodeLoggerTest::AddExtension(std::string_view extension) {
  std::transform(logfiles_.begin(), logfiles_.end(), logfiles_.begin(),
                 [extension](const std::string &in) {
                   return absl::StrCat(in, extension);
                 });

  std::transform(structured_logfiles_.begin(), structured_logfiles_.end(),
                 structured_logfiles_.begin(),
                 [extension](std::vector<std::string> in) {
                   std::transform(in.begin(), in.end(), in.begin(),
                                  [extension](const std::string &in_str) {
                                    return absl::StrCat(in_str, extension);
                                  });
                   return in;
                 });
}

std::vector<std::vector<std::string>> ToLogReaderVector(
    const std::vector<LogFile> &log_files) {
  std::vector<std::vector<std::string>> result;
  for (const LogFile &log_file : log_files) {
    for (const LogParts &log_parts : log_file.parts) {
      std::vector<std::string> parts;
      for (const std::string &part : log_parts.parts) {
        parts.emplace_back(part);
      }
      result.emplace_back(std::move(parts));
    }
  }
  return result;
}

std::vector<CompressionParams> SupportedCompressionAlgorithms() {
  return {{"",
           [](size_t max_message_size) {
             return std::make_unique<DummyEncoder>(max_message_size);
           }},
          {SnappyDecoder::kExtension,
           [](size_t max_message_size) {
             return std::make_unique<SnappyEncoder>(max_message_size, 32768);
           }},
#ifdef LZMA
          {LzmaDecoder::kExtension,
           [](size_t max_message_size) {
             return std::make_unique<LzmaEncoder>(max_message_size, 3);
           }}
#endif  // LZMA
  };
}

std::ostream &operator<<(std::ostream &ostream,
                         const CompressionParams &params) {
  ostream << "\"" << params.extension << "\"";
  return ostream;
}

std::ostream &operator<<(std::ostream &ostream, const ConfigParams &params) {
  ostream << "{config: \"" << params.config << "\", shared: " << params.shared
          << ", sha256: \"" << params.sha256 << "\", relogged_sha256: \""
          << params.relogged_sha256 << "\"}";
  return ostream;
}

std::vector<std::pair<std::vector<realtime_clock::time_point>,
                      std::vector<realtime_clock::time_point>>>
ConfirmReadable(const std::vector<std::string> &files,
                realtime_clock::time_point start_time,
                realtime_clock::time_point end_time) {
  {
    LogReader reader(SortParts(files));

    SimulatedEventLoopFactory log_reader_factory(reader.configuration());
    reader.Register(&log_reader_factory);

    log_reader_factory.Run();

    reader.Deregister();
  }
  {
    std::vector<std::pair<std::vector<realtime_clock::time_point>,
                          std::vector<realtime_clock::time_point>>>
        result;
    LogReader reader(SortParts(files));

    reader.SetStartTime(start_time);
    reader.SetEndTime(end_time);

    SimulatedEventLoopFactory log_reader_factory(reader.configuration());
    reader.RegisterWithoutStarting(&log_reader_factory);
    result.resize(
        configuration::NodesCount(log_reader_factory.configuration()));
    if (configuration::MultiNode(log_reader_factory.configuration())) {
      size_t i = 0;
      for (const aos::Node *node :
           *log_reader_factory.configuration()->nodes()) {
        LOG(INFO) << "Registering start";
        reader.OnStart(node, [node, &log_reader_factory, &result,
                              node_index = i]() {
          LOG(INFO) << "Starting " << node->name()->string_view();
          result[node_index].first.push_back(
              log_reader_factory.GetNodeEventLoopFactory(node)->realtime_now());
        });
        reader.OnEnd(node, [node, &log_reader_factory, &result,
                            node_index = i]() {
          LOG(INFO) << "Ending " << node->name()->string_view();
          result[node_index].second.push_back(
              log_reader_factory.GetNodeEventLoopFactory(node)->realtime_now());
        });
        ++i;
      }
    } else {
      reader.OnStart([&log_reader_factory, &result]() {
        LOG(INFO) << "Starting";
        result[0].first.push_back(
            log_reader_factory.GetNodeEventLoopFactory(nullptr)
                ->realtime_now());
      });
      reader.OnEnd([&log_reader_factory, &result]() {
        LOG(INFO) << "Ending";
        result[0].second.push_back(
            log_reader_factory.GetNodeEventLoopFactory(nullptr)
                ->realtime_now());
      });
    }

    log_reader_factory.Run();

    reader.Deregister();

    for (auto x : result) {
      for (auto y : x.first) {
        VLOG(1) << "Start " << y;
      }
      for (auto y : x.second) {
        VLOG(1) << "End " << y;
      }
    }
    return result;
  }
}

// Counts the number of messages on a channel.  Returns (channel name, channel
// type, count) for every message matching matcher()
std::vector<std::tuple<std::string, std::string, int>> CountChannelsMatching(
    std::shared_ptr<const aos::Configuration> config, std::string_view filename,
    std::function<bool(const UnpackedMessageHeader *)> matcher) {
  MessageReader message_reader(filename);
  std::vector<int> counts(config->channels()->size(), 0);

  while (true) {
    std::shared_ptr<UnpackedMessageHeader> msg = message_reader.ReadMessage();
    if (!msg) {
      break;
    }

    if (matcher(msg.get())) {
      counts[msg->channel_index]++;
    }
  }

  std::vector<std::tuple<std::string, std::string, int>> result;
  int channel = 0;
  for (size_t i = 0; i < counts.size(); ++i) {
    if (counts[i] != 0) {
      const Channel *channel = config->channels()->Get(i);
      result.push_back(std::make_tuple(channel->name()->str(),
                                       channel->type()->str(), counts[i]));
    }
    ++channel;
  }

  return result;
}

// Counts the number of messages (channel, count) for all data messages.
std::vector<std::tuple<std::string, std::string, int>> CountChannelsData(
    std::shared_ptr<const aos::Configuration> config,
    std::string_view filename) {
  return CountChannelsMatching(
      config, filename, [](const UnpackedMessageHeader *msg) {
        if (msg->span.data() != nullptr) {
          CHECK(!msg->monotonic_remote_time.has_value());
          CHECK(!msg->realtime_remote_time.has_value());
          CHECK(!msg->remote_queue_index.has_value());
          return true;
        }
        return false;
      });
}

// Counts the number of messages (channel, count) for all timestamp messages.
std::vector<std::tuple<std::string, std::string, int>> CountChannelsTimestamp(
    std::shared_ptr<const aos::Configuration> config,
    std::string_view filename) {
  return CountChannelsMatching(
      config, filename, [](const UnpackedMessageHeader *msg) {
        if (msg->span.data() == nullptr) {
          CHECK(msg->monotonic_remote_time.has_value());
          CHECK(msg->realtime_remote_time.has_value());
          CHECK(msg->remote_queue_index.has_value());
          return true;
        }
        return false;
      });
}

}  // namespace testing
}  // namespace logger
}  // namespace aos
