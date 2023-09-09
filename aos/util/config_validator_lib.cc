#include "aos/util/config_validator_lib.h"

#include <chrono>

#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/timestamp_channel.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/simulation_logger.h"

DECLARE_bool(validate_timestamp_logger_nodes);

namespace aos::util {

namespace {
void RunSimulationAndExit(const aos::Configuration *config) {
  aos::SimulatedEventLoopFactory factory(config);

  factory.RunFor(std::chrono::seconds(1));

  std::exit(EXIT_SUCCESS);
}

// Checks if either the node is in the specified list of node names or if the
// list is empty (in which case it is treated as matching all nodes).
bool NodeInList(
    const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> *list,
    const aos::Node *node) {
  if (list == nullptr || list->size() == 0) {
    return true;
  }
  for (const flatbuffers::String *name : *list) {
    if (name->string_view() == node->name()->string_view()) {
      return true;
    }
  }
  return false;
}

}  // namespace

void ConfigIsValid(const aos::Configuration *config,
                   const ConfigValidatorConfig *validation_config) {
  ASSERT_TRUE(config->has_channels())
      << "An AOS config must have channels. If you have a valid use-case for "
         "channels with no channels, please write a design proposal.";

  // First, we do some sanity checks--these are likely to indicate a malformed
  // config, and so catching them early with a clear error message is likely to
  // help.

  // The set of all channels that are required by the channels that are
  // configured--these are the remote timestamp channels that *must* be present,
  // and ideally there are no other channels present.
  std::set<const Channel *> required_timestamp_channels;
  // The set of all channels that *look* like remote timestamp channels. This
  // may include channels that are improperly configured and thus have typos &
  // aren't actually going to do anything at runtime.
  std::set<const Channel *> configured_timestamp_channels;
  bool validation_failed = false;
  for (size_t channel_index = 0; channel_index < config->channels()->size();
       ++channel_index) {
    const aos::Channel *channel = config->channels()->Get(channel_index);
    ASSERT_TRUE(channel->has_name()) << "All AOS channels must have a name.";
    ASSERT_TRUE(channel->has_type()) << "All AOS channels must have a type.";

    const bool channel_looks_like_remote_message_channel =
        channel->type()->string_view() ==
        message_bridge::RemoteMessage::GetFullyQualifiedName();

    const bool check_for_not_logged_channels =
        !validation_config->has_logging() ||
        validation_config->logging()->all_channels_logged();
    const bool channel_is_not_logged =
        channel->logger() == aos::LoggerConfig::NOT_LOGGED;
    if (check_for_not_logged_channels) {
      if (channel_looks_like_remote_message_channel != channel_is_not_logged) {
        LOG(WARNING)
            << "Channel " << configuration::StrippedChannelToString(channel)
            << " is " << EnumNameLoggerConfig(channel->logger()) << " but "
            << (channel_looks_like_remote_message_channel ? "is" : "is not")
            << " a remote timestamp channel. This is almost certainly wrong.";
        validation_failed = true;
      }
    }

    if (channel_looks_like_remote_message_channel) {
      configured_timestamp_channels.insert(channel);
    } else {
      if (channel->has_destination_nodes()) {
        // TODO(james): Technically the timestamp finder should receive a
        // non-empty application name. However, there are no known users that
        // care at this moment.
        message_bridge::ChannelTimestampFinder timestamp_finder(
            config, "",
            configuration::GetNode(config,
                                   channel->source_node()->string_view()));
        for (const Connection *connection : *channel->destination_nodes()) {
          switch (connection->timestamp_logger()) {
            case LoggerConfig::NOT_LOGGED:
            case LoggerConfig::LOCAL_LOGGER:
              if (connection->has_timestamp_logger_nodes()) {
                LOG(WARNING)
                    << "Connections that are "
                    << EnumNameLoggerConfig(connection->timestamp_logger())
                    << " should not have remote timestamp logger nodes "
                       "populated. This is for the connection to "
                    << connection->name()->string_view() << " on "
                    << configuration::StrippedChannelToString(channel);
                validation_failed = true;
              }
              break;
            case LoggerConfig::REMOTE_LOGGER:
            case LoggerConfig::LOCAL_AND_REMOTE_LOGGER:
              if (!connection->has_timestamp_logger_nodes() ||
                  connection->timestamp_logger_nodes()->size() != 1 ||
                  connection->timestamp_logger_nodes()->Get(0)->string_view() !=
                      channel->source_node()->string_view()) {
                LOG(WARNING)
                    << "Connections that are "
                    << EnumNameLoggerConfig(connection->timestamp_logger())
                    << " should have exactly 1 remote timestamp logger node "
                       "populated, and that node should be the source_node ("
                    << channel->source_node()->string_view()
                    << "). This is for the connection to "
                    << connection->name()->string_view() << " on "
                    << configuration::StrippedChannelToString(channel);
                validation_failed = true;
              }
              // TODO(james): This will be overly noisy, as it ends up
              // CHECK-failing.
              required_timestamp_channels.insert(CHECK_NOTNULL(
                  timestamp_finder.ForChannel(channel, connection)));
              break;
          }
        }
      }
    }
  }

  // Check that all of the things that look like timestamp channels are indeed
  // required.
  // Note: Because ForChannel() will die if a required channel is not present,
  // we do not do a separate check that all the required channels exist.
  for (const auto &channel : configured_timestamp_channels) {
    if (required_timestamp_channels.count(channel) == 0) {
      LOG(WARNING) << "Timestamp channel "
                   << configuration::StrippedChannelToString(channel)
                   << " was specified in the config but is not used.";
      validation_failed = true;
    }
  }

  if (validation_failed) {
    FAIL() << "Remote timestamp linting failed.";
    return;
  }

  // Because the most common way for simulation to fail involves it dying, force
  // it to fail in a slightly more controlled manner.
  ASSERT_EXIT(RunSimulationAndExit(config),
              ::testing::ExitedWithCode(EXIT_SUCCESS), "");

  if (!validation_config->has_logging() || !configuration::MultiNode(config)) {
    return;
  }

  // We will run all the logger configs in two modes:
  // 1) We don't send any data on any non-infrastructure channels; this confirms
  //    that the logs are readable in the absence of any user applications being
  //    present.
  // 2) We confirm that we can generate a good logfile that actually has data
  //    on every channel (some checks in the LogReader may not get hit if there
  //    is no data on a given channel).
  const std::string log_path = aos::testing::TestTmpDir() + "/logs/";
  for (const bool send_data_on_channels : {false, true}) {
    SCOPED_TRACE(send_data_on_channels);
    // Single nodes (multi-nodes with node count = 1) will not produce readable
    // logs in the absense of data.
    if (!send_data_on_channels && (configuration::NodesCount(config) == 1u)) {
      continue;
    }
    // Send timing report when we are sending data.
    const bool do_skip_timing_report = !send_data_on_channels;
    for (const LoggerNodeSetValidation *logger_set :
         *validation_config->logging()->logger_sets()) {
      SCOPED_TRACE(aos::FlatbufferToJson(logger_set));
      aos::SimulatedEventLoopFactory factory(config);
      std::vector<std::unique_ptr<LoggerState>> loggers;
      if (logger_set->has_loggers() && logger_set->loggers()->size() > 0) {
        std::vector<std::string> logger_nodes;
        for (const auto &node : *logger_set->loggers()) {
          logger_nodes.push_back(node->str());
        }
        loggers = MakeLoggersForNodes(&factory, logger_nodes, log_path,
                                      do_skip_timing_report);
      } else {
        loggers =
            MakeLoggersForAllNodes(&factory, log_path, do_skip_timing_report);
      }

      std::vector<std::unique_ptr<EventLoop>> test_loops;
      std::map<std::string, std::vector<std::unique_ptr<RawSender>>>
          test_senders;

      if (send_data_on_channels) {
        // Make a sender on every non-infrastructure channel on every node
        // (including channels that may not be observable by the current logger
        // set).
        for (const aos::Node *node : configuration::GetNodes(config)) {
          test_loops.emplace_back(factory.MakeEventLoop("", node));
          for (const aos::Channel *channel : *config->channels()) {
            // TODO(james): Make a more sophisticated check for "infrastructure"
            // channels than just looking for a "/aos" in the channel--we don't
            // accidentally want to spam nonsense data onto any timestamp
            // channels, though.
            if (configuration::ChannelIsSendableOnNode(channel, node) &&
                channel->name()->str().find("/aos") == std::string::npos &&
                channel->logger() != LoggerConfig::NOT_LOGGED) {
              test_senders[node->name()->str()].emplace_back(
                  test_loops.back()->MakeRawSender(channel));
              RawSender *sender =
                  test_senders[node->name()->str()].back().get();
              test_loops.back()->OnRun([sender, channel]() {
                flatbuffers::DetachedBuffer buffer =
                    JsonToFlatbuffer("{}", channel->schema());
                sender->CheckOk(sender->Send(buffer.data(), buffer.size()));
              });
            }
          }
        }
      }

      factory.RunFor(std::chrono::seconds(2));

      // Get all of the loggers to close before trying to read the logfiles.
      loggers.clear();

      // Confirm that we can read the log, and that if we put data in it that we
      // can find data on all the nodes that the user cares about.
      logger::LogReader reader(logger::SortParts(logger::FindLogs(log_path)));
      SimulatedEventLoopFactory replay_factory(reader.configuration());
      reader.RegisterWithoutStarting(&replay_factory);

      // Find every channel we deliberately sent data on, and if it is for a
      // node that we care about, confirm that we get it during replay.
      std::vector<std::unique_ptr<EventLoop>> replay_loops;
      std::vector<std::unique_ptr<RawFetcher>> fetchers;
      for (const aos::Node *node :
           configuration::GetNodes(replay_factory.configuration())) {
        // If the user doesn't care about this node, don't check it.
        if (!NodeInList(logger_set->replay_nodes(), node)) {
          continue;
        }
        replay_loops.emplace_back(replay_factory.MakeEventLoop("", node));
        for (const auto &sender : test_senders[node->name()->str()]) {
          const aos::Channel *channel = configuration::GetChannel(
              replay_factory.configuration(), sender->channel(), "", node);
          fetchers.emplace_back(replay_loops.back()->MakeRawFetcher(channel));
        }
      }

      std::vector<std::pair<const aos::Node *, std::unique_ptr<RawFetcher>>>
          remote_fetchers;
      for (const auto &fetcher : fetchers) {
        for (auto &loop : replay_loops) {
          const Connection *connection =
              configuration::ConnectionToNode(fetcher->channel(), loop->node());
          if (connection != nullptr) {
            remote_fetchers.push_back(std::make_pair(
                loop->node(), loop->MakeRawFetcher(fetcher->channel())));
          }
        }
      }

      replay_factory.Run();

      for (auto &fetcher : fetchers) {
        EXPECT_TRUE(fetcher->Fetch())
            << "Failed to log or replay any data on "
            << configuration::StrippedChannelToString(fetcher->channel());
      }

      for (auto &pair : remote_fetchers) {
        EXPECT_TRUE(pair.second->Fetch())
            << "Failed to log or replay any data on "
            << configuration::StrippedChannelToString(pair.second->channel())
            << " from remote node " << logger::MaybeNodeName(pair.first) << ".";
      }

      reader.Deregister();

      // Clean up the logs.
      UnlinkRecursive(log_path);
    }
  }
}

}  // namespace aos::util
