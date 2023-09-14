
#include "aos/events/logging/logfile_validator.h"

#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logfile_validator.h"
#include "aos/network/multinode_timestamp_filter.h"

namespace aos::logger {
bool MultiNodeLogIsReadable(const LogFilesContainer &log_files,
                            bool skip_order_validation) {
  const Configuration *config = log_files.config().get();

  CHECK(configuration::MultiNode(config))
      << ": Timestamps only make sense in a multi-node world.";

  // Now, build up all the TimestampMapper classes to read and sort the data.
  std::vector<std::unique_ptr<TimestampMapper>> mappers;

  for (const Node *node : configuration::GetNodes(config)) {
    auto node_name = MaybeNodeName(node);
    // Confirm that all the parts are from the same boot if there are enough
    // parts to not be from the same boot.
    if (log_files.ContainsPartsForNode(node_name)) {
      // Filter the parts relevant to each node when building the mapper.
      mappers.emplace_back(std::make_unique<TimestampMapper>(
          node_name, log_files, TimestampQueueStrategy::kQueueTogether));
    } else {
      mappers.emplace_back(nullptr);
    }
  }

  // Now, build up the estimator used to solve for time.
  message_bridge::MultiNodeNoncausalOffsetEstimator multinode_estimator(
      config, config, log_files.boots(), skip_order_validation,
      std::chrono::seconds(0));
  multinode_estimator.set_reboot_found(
      [config](distributed_clock::time_point reboot_time,
               const std::vector<logger::BootTimestamp> &node_times) {
        VLOG(1) << "Rebooted at distributed " << reboot_time;
        size_t node_index = 0;
        for (const logger::BootTimestamp &time : node_times) {
          VLOG(1) << "  "
                  << config->nodes()->Get(node_index)->name()->string_view()
                  << " " << time;
          ++node_index;
        }
      });

  // Because RAII doesn't let us do non-fatal/non-exception things, use this
  // when returning to handle certain cleanup-related checks that would normally
  // happen fatally in the estimator destrictor.
  auto preempt_destructor = [&multinode_estimator](bool success) {
    if (!multinode_estimator.RunDestructorChecks()) {
      return false;
    }
    return success;
  };

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
    VLOG(1) << "Reading all data for " << node->name()->string_view();
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
  std::optional<std::optional<const std::tuple<distributed_clock::time_point,
                                               std::vector<BootTimestamp>> *>>
      next_timestamp = multinode_estimator.QueueNextTimestamp();
  if (!next_timestamp.has_value() || !next_timestamp.value().has_value()) {
    return preempt_destructor(false);
  }
  VLOG(1) << "Starting at:";
  for (const Node *node : configuration::GetNodes(config)) {
    const size_t node_index = configuration::GetNodeIndex(config, node);
    VLOG(1) << "  " << node->name()->string_view() << " -> "
            << std::get<1>(*next_timestamp.value().value())[node_index].time;
  }

  std::vector<monotonic_clock::time_point> just_monotonic(
      std::get<1>(*next_timestamp.value().value()).size());
  for (size_t i = 0; i < just_monotonic.size(); ++i) {
    CHECK_EQ(std::get<1>(*next_timestamp.value().value())[i].boot, 0u);
    just_monotonic[i] = std::get<1>(*next_timestamp.value().value())[i].time;
  }
  multinode_estimator.Start(just_monotonic);

  // As we pull off all the timestamps, the time problem is continually solved,
  // filling in the CSV files.
  while (true) {
    std::optional<std::optional<const std::tuple<distributed_clock::time_point,
                                                 std::vector<BootTimestamp>> *>>
        next_timestamp = multinode_estimator.QueueNextTimestamp();
    if (!next_timestamp.has_value()) {
      return preempt_destructor(false);
    }
    if (!next_timestamp.value().has_value()) {
      break;
    }
    multinode_estimator.ObserveTimePassed(
        std::get<0>(*next_timestamp.value().value()));
  }

  VLOG(1) << "Done";

  return preempt_destructor(true);
}

bool LogIsReadableIfMultiNode(const LogFilesContainer &log_files) {
  if (aos::configuration::NodesCount(log_files.config().get()) == 1u) {
    return true;
  }
  return MultiNodeLogIsReadable(log_files);
}
}  // namespace aos::logger
