#include <iostream>
#include <string>
#include <vector>

#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/init.h"
#include "aos/network/multinode_timestamp_filter.h"
#include "gflags/gflags.h"

// This is a simple application to match up data with timestamps for a node in a
// log.  It doesn't solve the timestamp problem, but is still quite useful for
// debugging what happened in a log.

DEFINE_string(node, "", "The node to dump sorted messages for");

namespace aos::logger {

namespace chrono = std::chrono;

std::string LogFileVectorToString(std::vector<logger::LogFile> log_files) {
  std::stringstream ss;
  for (const auto &f : log_files) {
    ss << f << "\n";
  }
  return ss.str();
}

int Main(int argc, char **argv) {
  const std::vector<std::string> unsorted_logfiles = FindLogs(argc, argv);
  const std::vector<LogFile> log_files = SortParts(unsorted_logfiles);

  CHECK_GT(log_files.size(), 0u);
  // Validate that we have the same config everwhere.  This will be true if
  // all the parts were sorted together and the configs match.
  const Configuration *config = nullptr;
  for (const LogFile &log_file : log_files) {
    VLOG(1) << log_file;
    if (config == nullptr) {
      config = log_file.config.get();
    } else {
      CHECK_EQ(config, log_file.config.get());
    }
  }

  // Haven't tested this on a single node log, and don't really see a need to
  // right now.  The higher layers just work.
  CHECK(configuration::MultiNode(config));

  // Now, build up all the TimestampMapper classes to read and sort the data.
  std::vector<std::unique_ptr<TimestampMapper>> mappers;
  TimestampMapper *node_mapper = nullptr;

  for (const Node *node : configuration::GetNodes(config)) {
    std::vector<LogParts> filtered_parts =
        FilterPartsForNode(log_files, node->name()->string_view());

    // Confirm that all the parts are from the same boot if there are enough
    // parts to not be from the same boot.
    if (!filtered_parts.empty()) {
      // Filter the parts relevant to each node when building the mapper.
      mappers.emplace_back(
          std::make_unique<TimestampMapper>(std::move(filtered_parts)));
      if (node->name()->string_view() == FLAGS_node) {
        node_mapper = mappers.back().get();
      }
    } else {
      mappers.emplace_back(nullptr);
    }
  }

  CHECK(node_mapper != nullptr) << ": Failed to find node " << FLAGS_node;

  // Hook the peers up so data gets matched.
  for (std::unique_ptr<TimestampMapper> &mapper1 : mappers) {
    for (std::unique_ptr<TimestampMapper> &mapper2 : mappers) {
      if (mapper1.get() != mapper2.get() && mapper1 && mapper2) {
        mapper1->AddPeer(mapper2.get());
      }
    }
  }

  // Now, read all the timestamps for each node.  This is simpler than the
  // logger on purpose.  It loads in *all* the timestamps in 1 go per node,
  // ignoring memory usage.
  const Node *node = configuration::GetNode(config, FLAGS_node);

  LOG(INFO) << "Reading all data for " << node->name()->string_view();
  const size_t node_index = configuration::GetNodeIndex(config, node);
  TimestampMapper *timestamp_mapper = mappers[node_index].get();
  CHECK(timestamp_mapper != nullptr);
  CHECK_EQ(timestamp_mapper, node_mapper);

  while (true) {
    TimestampedMessage *m = timestamp_mapper->Front();
    if (m == nullptr) {
      break;
    }
    std::cout << config->nodes()
                     ->Get(configuration::GetNodeIndex(
                         config, config->channels()
                                     ->Get(m->channel_index)
                                     ->source_node()
                                     ->string_view()))
                     ->name()
                     ->string_view()
              << " "
              << configuration::StrippedChannelToString(
                     config->channels()->Get(m->channel_index))
              << " " << *m << "\n";
    timestamp_mapper->PopFront();
  }

  return 0;
}

}  // namespace aos::logger

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  return aos::logger::Main(argc, argv);
}
