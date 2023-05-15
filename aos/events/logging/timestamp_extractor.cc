#include <iostream>
#include <string>
#include <vector>

#include "gflags/gflags.h"

#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/init.h"
#include "aos/network/multinode_timestamp_filter.h"

DECLARE_bool(timestamps_to_csv);
DEFINE_bool(skip_order_validation, false,
            "If true, ignore any out of orderness in replay");

namespace aos::logger {

namespace chrono = std::chrono;

int Main(int argc, char **argv) {
  const std::vector<std::string> unsorted_logfiles = FindLogs(argc, argv);
  const LogFilesContainer log_files(SortParts(unsorted_logfiles));
  const Configuration *config = log_files.config();

  CHECK(configuration::MultiNode(config))
      << ": Timestamps only make sense in a multi-node world.";

  // Now, build up all the TimestampMapper classes to read and sort the data.
  std::vector<std::unique_ptr<TimestampMapper>> mappers;

  for (const Node *node : configuration::GetNodes(config)) {
    auto node_name = MaybeNodeName(node);
    // Confirm that all the parts are from the same boot if there are enough
    // parts to not be from the same boot.
    if (!log_files.ContainsPartsForNode(node_name)) {
      // Filter the parts relevant to each node when building the mapper.
      mappers.emplace_back(
          std::make_unique<TimestampMapper>(node_name, log_files));
    } else {
      mappers.emplace_back(nullptr);
    }
  }

  // Now, build up the estimator used to solve for time.
  message_bridge::MultiNodeNoncausalOffsetEstimator multinode_estimator(
      config, config, log_files.front_boots(), FLAGS_skip_order_validation,
      chrono::seconds(0));
  multinode_estimator.set_reboot_found(
      [config](distributed_clock::time_point reboot_time,
               const std::vector<logger::BootTimestamp> &node_times) {
        LOG(INFO) << "Rebooted at distributed " << reboot_time;
        size_t node_index = 0;
        for (const logger::BootTimestamp &time : node_times) {
          LOG(INFO) << "  "
                    << config->nodes()->Get(node_index)->name()->string_view()
                    << " " << time;
          ++node_index;
        }
      });

  {
    std::vector<TimestampMapper *> timestamp_mappers;
    for (std::unique_ptr<TimestampMapper> &mapper : mappers) {
      timestamp_mappers.emplace_back(mapper.get());
    }
    multinode_estimator.SetTimestampMappers(std::move(timestamp_mappers));
  }

  // To make things more like the logger and faster, cache the node + channel ->
  // filter mapping in a set of vectors.
  std::vector<std::vector<message_bridge::NoncausalOffsetEstimator *>> filters;
  filters.resize(configuration::NodesCount(config));

  for (const Node *node : configuration::GetNodes(config)) {
    const size_t node_index = configuration::GetNodeIndex(config, node);
    filters[node_index].resize(config->channels()->size(), nullptr);
    for (size_t channel_index = 0; channel_index < config->channels()->size();
         ++channel_index) {
      const Channel *channel = config->channels()->Get(channel_index);

      if (!configuration::ChannelIsSendableOnNode(channel, node) &&
          configuration::ChannelIsReadableOnNode(channel, node)) {
        // We've got a message which is being forwarded to this node.
        const Node *source_node = configuration::GetNode(
            config, channel->source_node()->string_view());
        filters[node_index][channel_index] =
            multinode_estimator.GetFilter(node, source_node);
      }
    }
  }

  multinode_estimator.CheckGraph();

  // Now, read all the timestamps for each node.  This is simpler than the
  // logger on purpose.  It loads in *all* the timestamps in 1 go per node,
  // ignoring memory usage.
  for (const Node *node : configuration::GetNodes(config)) {
    LOG(INFO) << "Reading all data for " << node->name()->string_view();
    const size_t node_index = configuration::GetNodeIndex(config, node);
    TimestampMapper *timestamp_mapper = mappers[node_index].get();
    if (timestamp_mapper == nullptr) {
      continue;
    }
    while (true) {
      TimestampedMessage *m = timestamp_mapper->Front();
      if (m == nullptr) {
        break;
      }
      timestamp_mapper->PopFront();
    }
  }

  // Don't get clever. Use the first time as the start time.  Note: this is
  // different than how log_cat and others work.
  std::optional<const std::tuple<distributed_clock::time_point,
                                 std::vector<BootTimestamp>> *>
      next_timestamp = multinode_estimator.QueueNextTimestamp();
  CHECK(next_timestamp);
  LOG(INFO) << "Starting at:";
  for (const Node *node : configuration::GetNodes(config)) {
    const size_t node_index = configuration::GetNodeIndex(config, node);
    LOG(INFO) << "  " << node->name()->string_view() << " -> "
              << std::get<1>(*next_timestamp.value())[node_index].time;
  }

  std::vector<monotonic_clock::time_point> just_monotonic(
      std::get<1>(*next_timestamp.value()).size());
  for (size_t i = 0; i < just_monotonic.size(); ++i) {
    CHECK_EQ(std::get<1>(*next_timestamp.value())[i].boot, 0u);
    just_monotonic[i] = std::get<1>(*next_timestamp.value())[i].time;
  }
  multinode_estimator.Start(just_monotonic);

  // As we pull off all the timestamps, the time problem is continually solved,
  // filling in the CSV files.
  while (true) {
    std::optional<const std::tuple<distributed_clock::time_point,
                                   std::vector<BootTimestamp>> *>
        next_timestamp = multinode_estimator.QueueNextTimestamp();
    if (!next_timestamp) {
      break;
    }
    multinode_estimator.ObserveTimePassed(std::get<0>(*next_timestamp.value()));
  }

  LOG(INFO) << "Done";

  return 0;
}

}  // namespace aos::logger

int main(int argc, char **argv) {
  FLAGS_timestamps_to_csv = true;
  gflags::SetUsageMessage(
      "Usage:\n"
      "  timestamp_extractor [args] logfile1 logfile2 ...\n\nThis program "
      "dumps out all the timestamps from a set of log files for plotting.  Use "
      "--skip_order_validation to skip any time estimation problems we find.");
  aos::InitGoogle(&argc, &argv);

  return aos::logger::Main(argc, argv);
}
