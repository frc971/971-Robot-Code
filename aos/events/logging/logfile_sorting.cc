#include "aos/events/logging/logfile_sorting.h"

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "aos/events/logging/logfile_utils.h"
#include "aos/flatbuffer_merge.h"
#include "aos/flatbuffers.h"
#include "aos/time/time.h"
#include "dirent.h"
#include "openssl/sha.h"
#include "sys/stat.h"

namespace aos {
namespace logger {
namespace chrono = std::chrono;

namespace {

// Check if string ends with ending
bool EndsWith(std::string_view str, std::string_view ending) {
  return str.size() >= ending.size() &&
         str.substr(str.size() - ending.size()) == ending;
}

bool FileExists(std::string filename) {
  struct stat stat_results;
  int error = stat(filename.c_str(), &stat_results);
  return error == 0;
}

bool ConfigOnly(const LogFileHeader *header) {
  CHECK_EQ(LogFileHeader::MiniReflectTypeTable()->num_elems, 17u);
  if (header->has_monotonic_start_time()) return false;
  if (header->has_realtime_start_time()) return false;
  if (header->has_max_out_of_order_duration()) return false;
  if (header->has_configuration_sha256()) return false;
  if (header->has_name()) return false;
  if (header->has_node()) return false;
  if (header->has_log_event_uuid()) return false;
  if (header->has_logger_instance_uuid()) return false;
  if (header->has_logger_node_boot_uuid()) return false;
  if (header->has_source_node_boot_uuid()) return false;
  if (header->has_logger_monotonic_start_time()) return false;
  if (header->has_logger_realtime_start_time()) return false;
  if (header->has_log_start_uuid()) return false;
  if (header->has_parts_uuid()) return false;
  if (header->has_parts_index()) return false;
  if (header->has_logger_node()) return false;

  return header->has_configuration();
}

}  // namespace

void FindLogs(std::vector<std::string> *files, std::string filename) {
  DIR *directory = opendir(filename.c_str());

  if (directory == nullptr) {
    if (EndsWith(filename, ".bfbs") || EndsWith(filename, ".bfbs.xz")) {
      files->emplace_back(filename);
    }
    return;
  }

  struct dirent *directory_entry;
  while ((directory_entry = readdir(directory)) != nullptr) {
    std::string next_filename = directory_entry->d_name;
    if (next_filename == "." || next_filename == "..") {
      continue;
    }

    std::string path = filename + "/" + next_filename;
    FindLogs(files, path);
  }

  closedir(directory);
}

std::vector<std::string> FindLogs(std::string filename) {
  std::vector<std::string> files;
  FindLogs(&files, filename);
  return files;
}

std::vector<std::string> FindLogs(int argc, char **argv) {
  std::vector<std::string> found_logfiles;

  for (int i = 1; i < argc; i++) {
    std::string filename = argv[i];
    if (FileExists(filename)) {
      aos::logger::FindLogs(&found_logfiles, filename);
    } else {
      LOG(FATAL) << "File " << filename << " does not exist";
    }
  }
  return found_logfiles;
}

// Start by grouping all parts by UUID, and extracting the part index.
// Datastructure to hold all the info extracted from a set of parts which go
// together so we can sort them afterwords.
struct UnsortedLogParts {
  // Start times.
  aos::monotonic_clock::time_point monotonic_start_time;
  aos::realtime_clock::time_point realtime_start_time;

  aos::monotonic_clock::time_point logger_monotonic_start_time =
      aos::monotonic_clock::min_time;
  aos::realtime_clock::time_point logger_realtime_start_time =
      aos::realtime_clock::min_time;

  // Node to save.
  std::string node;

  // The boot UUID of the node which generated this data.
  std::string source_boot_uuid;

  // Pairs of the filename and the part index for sorting.
  std::vector<std::pair<std::string, int>> parts;

  std::string config_sha256;
};

// Struct to hold both the node, and the parts associated with it.
struct UnsortedLogPartsMap {
  std::string logger_node;
  // The boot UUID of the node this log file was created on.
  std::string logger_boot_uuid;

  aos::monotonic_clock::time_point monotonic_start_time =
      aos::monotonic_clock::min_time;
  aos::realtime_clock::time_point realtime_start_time =
      aos::realtime_clock::min_time;

  // Name from a log.  All logs below have been confirmed to match.
  std::string name;

  std::map<std::string, UnsortedLogParts> unsorted_parts;
};

// Sort part files without UUIDs and part indexes as well.  Extract everything
// useful from the log in the first pass, then sort later.
struct UnsortedOldParts {
  // Part information with everything but the list of parts.
  LogParts parts;

  // Tuple of time for the data and filename needed for sorting after
  // extracting.
  std::vector<std::pair<monotonic_clock::time_point, std::string>>
      unsorted_parts;

  // Name from a log.  All logs below have been confirmed to match.
  std::string name;
};

struct NodeBootState {
  // Maps each boot to its constraining boots. If the bool in the value is true,
  // then the boot uuid in the key chronologically precedes the boot uuid in
  // the value, and vice versa if false.
  std::map<std::string, std::vector<std::pair<std::string, bool>>> constraints;

  // All the boots we know about.
  std::set<std::string> boots;
};

// Helper class to make it easier to sort a list of log files into
// std::vector<LogFile>
struct PartsSorter {
  // List of files that were corrupted.
  std::vector<std::string> corrupted;

  // Map from sha256 to the config.
  std::map<std::string, std::shared_ptr<const Configuration>>
      config_sha256_lookup;

  // Map holding the log_event_uuid -> second map.  The second map holds the
  // parts_uuid -> list of parts for sorting.
  std::map<std::string, UnsortedLogPartsMap> parts_list;

  // A list of all the old parts which we don't know how to sort using uuids.
  // There are enough of these in the wild that this is worth supporting.
  std::vector<UnsortedOldParts> old_parts;

  // Populates the class's datastructures from the input file list.
  void PopulateFromFiles(const std::vector<std::string> &parts);

  // Wrangle parts_list into a map of boot uuids -> boot counts.
  std::map<std::string, int> ComputeBootCounts();

  // Reformats old_parts into a list of logfiles and returns it.  This destroys
  // state in PartsSorter.
  std::vector<LogFile> FormatOldParts();

  // Reformats parts_list into a list of logfiles and returns it.  This destroys
  // state in PartsSorter.
  std::vector<LogFile> FormatNewParts();
};

void PartsSorter::PopulateFromFiles(const std::vector<std::string> &parts) {
  // Now extract everything into our datastructures above for sorting.
  for (const std::string &part : parts) {
    std::optional<SizePrefixedFlatbufferVector<LogFileHeader>> log_header =
        ReadHeader(part);
    if (!log_header) {
      LOG(WARNING) << "Skipping " << part << " without a header";
      corrupted.emplace_back(part);
      continue;
    }

    const monotonic_clock::time_point monotonic_start_time(
        chrono::nanoseconds(log_header->message().monotonic_start_time()));
    const realtime_clock::time_point realtime_start_time(
        chrono::nanoseconds(log_header->message().realtime_start_time()));
    const monotonic_clock::time_point logger_monotonic_start_time(
        chrono::nanoseconds(
            log_header->message().logger_monotonic_start_time()));
    const realtime_clock::time_point logger_realtime_start_time(
        chrono::nanoseconds(
            log_header->message().logger_realtime_start_time()));

    const std::string_view node =
        log_header->message().has_node()
            ? log_header->message().node()->name()->string_view()
            : "";

    const std::string_view name =
        log_header->message().has_name()
            ? log_header->message().name()->string_view()
            : "";

    const std::string_view logger_node =
        log_header->message().has_logger_node()
            ? log_header->message().logger_node()->name()->string_view()
            : "";

    const std::string_view logger_boot_uuid =
        log_header->message().has_logger_node_boot_uuid()
            ? log_header->message().logger_node_boot_uuid()->string_view()
            : "";

    const std::string_view source_boot_uuid =
        log_header->message().has_source_node_boot_uuid()
            ? log_header->message().source_node_boot_uuid()->string_view()
            : "";

    const std::string_view configuration_sha256 =
        log_header->message().has_configuration_sha256()
            ? log_header->message().configuration_sha256()->string_view()
            : "";

    if (ConfigOnly(&log_header->message())) {
      const std::string hash = Sha256(log_header->span());

      if (config_sha256_lookup.find(hash) == config_sha256_lookup.end()) {
        auto header =
            std::make_shared<SizePrefixedFlatbufferVector<LogFileHeader>>(
                std::move(*log_header));
        config_sha256_lookup.emplace(
            hash, std::shared_ptr<const Configuration>(
                      header, header->message().configuration()));
      }
      continue;
    }

    if (configuration_sha256.empty()) {
      CHECK(log_header->message().has_configuration())
          << ": Failed to find header on " << part;
    } else {
      CHECK(!log_header->message().has_configuration())
          << ": Found header where one shouldn't be on " << part;
    }

    // Looks like an old log.  No UUID, index, and also single node.  We have
    // little to no multi-node log files in the wild without part UUIDs and
    // indexes which we care much about.
    if (!log_header->message().has_parts_uuid() &&
        !log_header->message().has_parts_index() &&
        !log_header->message().has_node()) {
      std::optional<SizePrefixedFlatbufferVector<MessageHeader>> first_message =
          ReadNthMessage(part, 0);
      if (!first_message) {
        LOG(WARNING) << "Skipping " << part << " without any messages";
        corrupted.emplace_back(part);
        continue;
      }
      const monotonic_clock::time_point first_message_time(
          chrono::nanoseconds(first_message->message().monotonic_sent_time()));

      // Find anything with a matching start time.  They all go together.
      auto result = std::find_if(
          old_parts.begin(), old_parts.end(),
          [&](const UnsortedOldParts &parts) {
            return parts.parts.monotonic_start_time == monotonic_start_time &&
                   parts.parts.realtime_start_time == realtime_start_time;
          });

      if (result == old_parts.end()) {
        old_parts.emplace_back();
        old_parts.back().parts.monotonic_start_time = monotonic_start_time;
        old_parts.back().parts.realtime_start_time = realtime_start_time;
        old_parts.back().unsorted_parts.emplace_back(
            std::make_pair(first_message_time, part));
        old_parts.back().name = name;
      } else {
        result->unsorted_parts.emplace_back(
            std::make_pair(first_message_time, part));
        CHECK_EQ(result->name, name);
      }
      continue;
    }

    CHECK(log_header->message().has_log_event_uuid());
    CHECK(log_header->message().has_parts_uuid());
    CHECK(log_header->message().has_parts_index());

    CHECK_EQ(log_header->message().has_logger_node(),
             log_header->message().has_node());

    const std::string log_event_uuid =
        log_header->message().log_event_uuid()->str();
    const std::string parts_uuid = log_header->message().parts_uuid()->str();
    int32_t parts_index = log_header->message().parts_index();

    auto log_it = parts_list.find(log_event_uuid);
    if (log_it == parts_list.end()) {
      log_it =
          parts_list
              .insert(std::make_pair(log_event_uuid, UnsortedLogPartsMap()))
              .first;
      log_it->second.logger_node = logger_node;
      log_it->second.logger_boot_uuid = logger_boot_uuid;
      log_it->second.name = name;
    } else {
      CHECK_EQ(log_it->second.logger_node, logger_node);
      CHECK_EQ(log_it->second.logger_boot_uuid, logger_boot_uuid);
      CHECK_EQ(log_it->second.name, name);
    }

    if (node == log_it->second.logger_node) {
      if (log_it->second.monotonic_start_time ==
          aos::monotonic_clock::min_time) {
        log_it->second.monotonic_start_time = monotonic_start_time;
        log_it->second.realtime_start_time = realtime_start_time;
      } else {
        CHECK_EQ(log_it->second.monotonic_start_time, monotonic_start_time);
        CHECK_EQ(log_it->second.realtime_start_time, realtime_start_time);
      }
    }

    auto it = log_it->second.unsorted_parts.find(parts_uuid);
    if (it == log_it->second.unsorted_parts.end()) {
      it = log_it->second.unsorted_parts
               .insert(std::make_pair(parts_uuid, UnsortedLogParts()))
               .first;
      it->second.monotonic_start_time = monotonic_start_time;
      it->second.realtime_start_time = realtime_start_time;
      it->second.logger_monotonic_start_time = logger_monotonic_start_time;
      it->second.logger_realtime_start_time = logger_realtime_start_time;
      it->second.node = std::string(node);
      it->second.source_boot_uuid = source_boot_uuid;
      it->second.config_sha256 = configuration_sha256;
    } else {
      CHECK_EQ(it->second.source_boot_uuid, source_boot_uuid);
      CHECK_EQ(it->second.config_sha256, configuration_sha256);
    }

    // First part might be min_time.  If it is, try to put a better time on it.
    if (it->second.monotonic_start_time == monotonic_clock::min_time) {
      it->second.monotonic_start_time = monotonic_start_time;
      it->second.logger_monotonic_start_time = logger_monotonic_start_time;
      it->second.logger_realtime_start_time = logger_realtime_start_time;
    } else if (monotonic_start_time != monotonic_clock::min_time) {
      CHECK_EQ(it->second.monotonic_start_time, monotonic_start_time);
      CHECK_EQ(it->second.logger_monotonic_start_time,
               logger_monotonic_start_time);
      CHECK_EQ(it->second.logger_realtime_start_time,
               logger_realtime_start_time);
    }
    if (it->second.realtime_start_time == realtime_clock::min_time) {
      it->second.realtime_start_time = realtime_start_time;
    } else if (realtime_start_time != realtime_clock::min_time) {
      CHECK_EQ(it->second.realtime_start_time, realtime_start_time);
    }

    it->second.parts.emplace_back(std::make_pair(part, parts_index));
  }
}

std::vector<LogFile> PartsSorter::FormatOldParts() {
  // Now reformat old_parts to be in the right datastructure to report.
  std::map<std::string, std::shared_ptr<const Configuration>>
      copied_config_sha256;
  CHECK(!old_parts.empty());

  std::vector<LogFile> result;
  for (UnsortedOldParts &p : old_parts) {
    // Sort by the oldest message in each file.
    std::sort(p.unsorted_parts.begin(), p.unsorted_parts.end(),
              [](const std::pair<monotonic_clock::time_point, std::string> &a,
                 const std::pair<monotonic_clock::time_point, std::string> &b) {
                return a.first < b.first;
              });
    LogFile log_file;

    // We want to use a single Configuration flatbuffer for all the parts to
    // make downstream easier.  Since this is an old log, it doesn't have a
    // SHA256 in the header to rely on, so we need a way to detect duplicates.
    //
    // SHA256 is decently fast, so use that as a representative hash of the
    // header.
    auto header = std::make_shared<SizePrefixedFlatbufferVector<LogFileHeader>>(
        std::move(*ReadHeader(p.unsorted_parts[0].second)));

    // Do a recursive copy to normalize the flatbuffer.  Different
    // configurations can be built different ways, and can even have their
    // vtable out of order.  Don't think and just trigger a copy.
    FlatbufferDetachedBuffer<Configuration> config_copy =
        RecursiveCopyFlatBuffer(header->message().configuration());

    std::string config_copy_sha256 = Sha256(config_copy.span());

    auto it = copied_config_sha256.find(config_copy_sha256);
    if (it != copied_config_sha256.end()) {
      log_file.config = it->second;
    } else {
      std::shared_ptr<const Configuration> config(
          header, header->message().configuration());

      copied_config_sha256.emplace(std::move(config_copy_sha256), config);
      log_file.config = config;
    }

    for (std::pair<monotonic_clock::time_point, std::string> &f :
         p.unsorted_parts) {
      p.parts.parts.emplace_back(std::move(f.second));
    }
    p.parts.config = log_file.config;
    log_file.parts.emplace_back(std::move(p.parts));
    log_file.monotonic_start_time = log_file.parts[0].monotonic_start_time;
    log_file.realtime_start_time = log_file.parts[0].realtime_start_time;
    log_file.corrupted = corrupted;
    log_file.name = p.name;
    result.emplace_back(std::move(log_file));
  }

  return result;
}

std::map<std::string, int> PartsSorter::ComputeBootCounts() {
  std::map<std::string, NodeBootState> boot_constraints;

  // TODO(austin): This is the "old" way.  Once we have better ordering info in
  // headers, we should use it.
  for (std::pair<const std::string, UnsortedLogPartsMap> &logs : parts_list) {
    std::map<std::string,
             std::vector<std::pair<std::string, std::pair<int, int>>>>
        node_boot_parts_index_ranges;

    for (std::pair<const std::string, UnsortedLogParts> &parts :
         logs.second.unsorted_parts) {
      CHECK_GT(parts.second.parts.size(), 0u);

      // Track that this boot exists so we know the overall set of boots we need
      // to link.
      {
        auto node_boot_constraints_it =
            boot_constraints.find(parts.second.node);
        if (node_boot_constraints_it == boot_constraints.end()) {
          node_boot_constraints_it =
              boot_constraints
                  .insert(std::make_pair(parts.second.node, NodeBootState()))
                  .first;
        }
        node_boot_constraints_it->second.boots.insert(
            parts.second.source_boot_uuid);
      }

      // Since the logger node is guarenteed to be on a single boot, we can look
      // for reboots on the remote nodes and use the parts_index to track when
      // they reboot.
      //
      // All the parts files rotate at the same time, so the parts_index will
      // match.  We can track the min and max parts_index for a
      // source_node_boot_uuid and use that to order the boot uuids.
      auto it = node_boot_parts_index_ranges.find(parts.second.node);
      if (it == node_boot_parts_index_ranges.end()) {
        it =
            node_boot_parts_index_ranges
                .insert(std::make_pair(
                    parts.second.node,
                    std::vector<std::pair<std::string, std::pair<int, int>>>()))
                .first;
      }
      std::vector<std::pair<std::string, std::pair<int, int>>>
          &boot_parts_index_ranges = it->second;

      // Find any existing ranges to extend, or make a new one otherwise.
      auto parts_index_ranges_it = std::find_if(
          boot_parts_index_ranges.begin(), boot_parts_index_ranges.end(),
          [&](const std::pair<std::string, std::pair<int, int>> &val) {
            return val.first == parts.second.source_boot_uuid;
          });
      if (parts_index_ranges_it == boot_parts_index_ranges.end()) {
        boot_parts_index_ranges.emplace_back(
            std::make_pair(parts.second.source_boot_uuid,
                           std::make_pair(std::numeric_limits<int>::max(),
                                          std::numeric_limits<int>::min())));
        parts_index_ranges_it = boot_parts_index_ranges.end() - 1;
      }

      std::pair<int, int> &ranges = parts_index_ranges_it->second;

      for (const std::pair<std::string, int> &part : parts.second.parts) {
        ranges.first = std::min(ranges.first, part.second);
        ranges.second = std::max(ranges.second, part.second);
      }
    }

    // Now that we have ranges per node per boot, iterate through those and
    // populate boot_constraints.
    for (auto &boot_parts_index_ranges : node_boot_parts_index_ranges) {
      auto it = boot_constraints.find(boot_parts_index_ranges.first);
      if (it == boot_constraints.end()) {
        it = boot_constraints
                 .insert(std::make_pair(boot_parts_index_ranges.first,
                                        NodeBootState()))
                 .first;
      }

      // Sort the list by parts_index, look for overlaps, then insert
      // everything as constraints.
      std::sort(boot_parts_index_ranges.second.begin(),
                boot_parts_index_ranges.second.end(), [](auto a, auto b) {
                  return a.second.first < b.second.second;
                });

      std::map<std::string, std::vector<std::pair<std::string, bool>>>
          &per_node_boot_constraints = it->second.constraints;

      if (boot_parts_index_ranges.second.size() < 2) {
        VLOG(1) << "Found only one boot for this node in this log.";
        continue;
      }

      for (size_t i = 1; i < boot_parts_index_ranges.second.size(); ++i) {
        CHECK_LT(boot_parts_index_ranges.second[i - 1].second.second,
                 boot_parts_index_ranges.second[i].second.first)
            << ": Overlapping parts_index, please investigate";
        auto first_per_boot_constraints = per_node_boot_constraints.find(
            boot_parts_index_ranges.second[i - 1].first);
        if (first_per_boot_constraints == per_node_boot_constraints.end()) {
          first_per_boot_constraints =
              per_node_boot_constraints
                  .insert(std::make_pair(
                      boot_parts_index_ranges.second[i - 1].first,
                      std::vector<std::pair<std::string, bool>>()))
                  .first;
        }

        auto second_per_boot_constraints = per_node_boot_constraints.find(
            boot_parts_index_ranges.second[i].first);
        if (second_per_boot_constraints == per_node_boot_constraints.end()) {
          second_per_boot_constraints =
              per_node_boot_constraints
                  .insert(std::make_pair(
                      boot_parts_index_ranges.second[i].first,
                      std::vector<std::pair<std::string, bool>>()))
                  .first;
        }

        first_per_boot_constraints->second.emplace_back(
            std::make_pair(boot_parts_index_ranges.second[i].first, true));
        second_per_boot_constraints->second.emplace_back(
            std::make_pair(boot_parts_index_ranges.second[i - 1].first, false));
      }
    }
  }

  // Print out our discovered constraints on request.
  if (VLOG_IS_ON(2)) {
    for (const std::pair<const std::string, NodeBootState> &node_state :
         boot_constraints) {
      LOG(INFO) << "Node " << node_state.first;
      CHECK_GT(node_state.second.boots.size(), 0u)
          << ": Need a boot from each node.";

      for (const std::string &boot : node_state.second.boots) {
        LOG(INFO) << "  boot " << boot;
      }
      for (const std::pair<std::string,
                           std::vector<std::pair<std::string, bool>>>
               &constraints : node_state.second.constraints) {
        for (const std::pair<std::string, bool> &constraint :
             constraints.second) {
          if (constraint.second) {
            LOG(INFO) << constraints.first << " < " << constraint.first;
          } else {
            LOG(INFO) << constraints.first << " > " << constraint.first;
          }
        }
      }
    }
  }

  // And now walk the constraint graph we have generated to order the boots.
  // This doesn't need to catch all the cases, it just needs to report when it
  // fails.
  std::map<std::string, int> boot_count_map;

  for (std::pair<const std::string, NodeBootState> &node_state :
       boot_constraints) {
    CHECK_GT(node_state.second.boots.size(), 0u)
        << ": Need a boot from each node.";
    if (node_state.second.boots.size() == 1u) {
      boot_count_map.insert(
          std::make_pair(*node_state.second.boots.begin(), 0));
      continue;
    }

    // Now, walk the dag to find the earliest boot.  Pick a node and start
    // traversing.
    std::string current_boot = *node_state.second.boots.begin();
    size_t update_count = 0;

    while (true) {
      auto it = node_state.second.constraints.find(current_boot);
      if (it == node_state.second.constraints.end()) {
        LOG(WARNING) << "Unconnected boot in set > 1";
        break;
      }

      bool updated = false;
      for (const std::pair<std::string, bool> &constraint : it->second) {
        if (!constraint.second) {
          updated = true;
          current_boot = constraint.first;
          break;
        }
      }
      if (!updated) {
        break;
      }

      ++update_count;

      CHECK_LE(update_count, node_state.second.boots.size())
          << ": Found a cyclic boot graph, giving up.";
    }

    // Now, walk the tree the other way to actually sort the boots.
    // TODO(austin): We can probably drop sorted_boots and directly insert into
    // the map with some more careful book-keeping.
    std::vector<std::string> sorted_boots;
    sorted_boots.emplace_back(current_boot);

    update_count = 0;
    while (true) {
      auto it = node_state.second.constraints.find(current_boot);
      if (it == node_state.second.constraints.end()) {
        LOG(WARNING) << "Unconnected boot in set > 1";
        break;
      }

      // This is a bit simplistic.  If you have a dag which isn't just a list,
      // we aren't guarenteed to walk the tree correctly.
      //
      //  a < b
      //  a < c
      //  b < c
      //
      // That is rare enough in practice that we can CHECK and fix it if someone
      // produces a valid use case.
      bool updated = false;
      for (const std::pair<std::string, bool> &constraint : it->second) {
        if (constraint.second) {
          updated = true;
          current_boot = constraint.first;
          break;
        }
      }
      if (!updated) {
        break;
      }

      sorted_boots.emplace_back(current_boot);

      ++update_count;

      CHECK_LE(update_count, node_state.second.boots.size())
          << ": Found a cyclic boot graph, giving up.";
    }

    CHECK_EQ(sorted_boots.size(), node_state.second.boots.size())
        << ": Graph failed to reach all the nodes.";

    VLOG(1) << "Node " << node_state.first;
    size_t boot_count = 0;
    for (const std::string &boot : sorted_boots) {
      VLOG(1) << "  Boot " << boot;
      boot_count_map.insert(std::make_pair(std::move(boot), boot_count));
      ++boot_count;
    }
  }

  return boot_count_map;
}

std::vector<LogFile> PartsSorter::FormatNewParts() {
  const std::map<std::string, int> boot_counts = ComputeBootCounts();

  std::map<std::string, std::shared_ptr<const Configuration>>
      copied_config_sha256;
  // Now, sort them and produce the final vector form.
  std::vector<LogFile> result;
  result.reserve(parts_list.size());
  for (std::pair<const std::string, UnsortedLogPartsMap> &logs : parts_list) {
    LogFile new_file;
    new_file.log_event_uuid = logs.first;
    new_file.logger_node = logs.second.logger_node;
    new_file.logger_boot_uuid = logs.second.logger_boot_uuid;
    new_file.monotonic_start_time = logs.second.monotonic_start_time;
    new_file.realtime_start_time = logs.second.realtime_start_time;
    new_file.name = logs.second.name;
    new_file.corrupted = corrupted;
    bool seen_part = false;
    std::string config_sha256;
    for (std::pair<const std::string, UnsortedLogParts> &parts :
         logs.second.unsorted_parts) {
      LogParts new_parts;
      new_parts.monotonic_start_time = parts.second.monotonic_start_time;
      new_parts.realtime_start_time = parts.second.realtime_start_time;
      new_parts.logger_monotonic_start_time =
          parts.second.logger_monotonic_start_time;
      new_parts.logger_realtime_start_time =
          parts.second.logger_realtime_start_time;
      new_parts.log_event_uuid = logs.first;
      new_parts.source_boot_uuid = parts.second.source_boot_uuid;
      new_parts.parts_uuid = parts.first;
      new_parts.node = std::move(parts.second.node);

      {
        auto boot_count_it = boot_counts.find(new_parts.source_boot_uuid);
        CHECK(boot_count_it != boot_counts.end());
        new_parts.boot_count = boot_count_it->second;
      }

      std::sort(parts.second.parts.begin(), parts.second.parts.end(),
                [](const std::pair<std::string, int> &a,
                   const std::pair<std::string, int> &b) {
                  return a.second < b.second;
                });
      new_parts.parts.reserve(parts.second.parts.size());
      for (std::pair<std::string, int> &p : parts.second.parts) {
        new_parts.parts.emplace_back(std::move(p.first));
      }

      if (!parts.second.config_sha256.empty()) {
        // The easy case.  We've got a sha256 to point to, so go look it up.
        // Abort if it doesn't exist.
        auto it = config_sha256_lookup.find(parts.second.config_sha256);
        CHECK(it != config_sha256_lookup.end())
            << ": Failed to find a matching config with a SHA256 of "
            << parts.second.config_sha256;
        new_parts.config_sha256 = std::move(parts.second.config_sha256);
        new_parts.config = it->second;
        if (!seen_part) {
          new_file.config_sha256 = new_parts.config_sha256;
          new_file.config = new_parts.config;
          config_sha256 = new_file.config_sha256;
        } else {
          CHECK_EQ(config_sha256, new_file.config_sha256)
              << ": Mismatched configs in " << new_file;
        }
      } else {
        CHECK(config_sha256.empty())
            << ": Part " << new_parts
            << " is missing a sha256 but other parts have one.";
        if (!seen_part) {
          // We want to use a single Configuration flatbuffer for all the parts
          // to make downstream easier.  Since this is an old log, it doesn't
          // have a SHA256 in the header to rely on, so we need a way to detect
          // duplicates.
          //
          // SHA256 is decently fast, so use that as a representative hash of
          // the header.
          auto header =
              std::make_shared<SizePrefixedFlatbufferVector<LogFileHeader>>(
                  std::move(*ReadHeader(new_parts.parts[0])));

          // Do a recursive copy to normalize the flatbuffer.  Different
          // configurations can be built different ways, and can even have their
          // vtable out of order.  Don't think and just trigger a copy.
          FlatbufferDetachedBuffer<Configuration> config_copy =
              RecursiveCopyFlatBuffer(header->message().configuration());

          std::string config_copy_sha256 = Sha256(config_copy.span());

          auto it = copied_config_sha256.find(config_copy_sha256);
          if (it != copied_config_sha256.end()) {
            new_file.config = it->second;
          } else {
            std::shared_ptr<const Configuration> config(
                header, header->message().configuration());

            copied_config_sha256.emplace(std::move(config_copy_sha256), config);
            new_file.config = config;
          }
        }
        new_parts.config = new_file.config;
      }
      seen_part = true;

      new_file.parts.emplace_back(std::move(new_parts));
    }
    result.emplace_back(std::move(new_file));
  }
  return result;
}

std::vector<LogFile> SortParts(const std::vector<std::string> &parts) {
  PartsSorter sorter;
  sorter.PopulateFromFiles(parts);

  if (sorter.old_parts.empty() && sorter.parts_list.empty()) {
    if (parts.empty()) {
      return std::vector<LogFile>{};
    } else {
      LogFile log_file;
      log_file.corrupted = std::move(sorter.corrupted);
      return std::vector<LogFile>{log_file};
    }
  }
  CHECK_NE(sorter.old_parts.empty(), sorter.parts_list.empty())
      << ": Can't have a mix of old and new parts.";

  if (!sorter.old_parts.empty()) {
    return sorter.FormatOldParts();
  }

  return sorter.FormatNewParts();
}

std::vector<std::string> FindNodes(const std::vector<LogFile> &parts) {
  std::set<std::string> nodes;
  for (const LogFile &log_file : parts) {
    for (const LogParts &part : log_file.parts) {
      nodes.insert(part.node);
    }
  }
  std::vector<std::string> node_list;
  while (!nodes.empty()) {
    node_list.emplace_back(std::move(nodes.extract(nodes.begin()).value()));
  }
  return node_list;
}

std::vector<std::string> FindLoggerNodes(const std::vector<LogFile> &parts) {
  std::set<std::string> nodes;
  for (const LogFile &log_file : parts) {
    nodes.insert(log_file.logger_node);
  }
  std::vector<std::string> node_list;
  while (!nodes.empty()) {
    node_list.emplace_back(nodes.extract(nodes.begin()).value());
  }
  return node_list;
}

std::vector<LogParts> FilterPartsForNode(const std::vector<LogFile> &parts,
                                         std::string_view node) {
  std::vector<LogParts> result;
  for (const LogFile &log_file : parts) {
    for (const LogParts &part : log_file.parts) {
      if (part.node == node) {
        result.emplace_back(part);
      }
    }
  }
  return result;
}

std::ostream &operator<<(std::ostream &stream, const LogFile &file) {
  stream << "{\n";
  if (!file.log_event_uuid.empty()) {
    stream << " \"log_event_uuid\": \"" << file.log_event_uuid << "\",\n";
  }
  if (!file.logger_node.empty()) {
    stream << " \"logger_node\": \"" << file.logger_node << "\",\n";
  }
  if (!file.logger_boot_uuid.empty()) {
    stream << " \"logger_boot_uuid\": \"" << file.logger_boot_uuid << "\",\n";
  }
  stream << " \"config\": \"" << file.config.get() << "\"";
  if (!file.config_sha256.empty()) {
    stream << ",\n \"config_sha256\": \"" << file.config_sha256 << "\"";
  }
  stream << ",\n \"monotonic_start_time\": \"" << file.monotonic_start_time
         << "\",\n \"realtime_start_time\": \"" << file.realtime_start_time
         << "\",\n";
  stream << " \"parts\": [\n";
  for (size_t i = 0; i < file.parts.size(); ++i) {
    if (i != 0u) {
      stream << ",\n";
    }
    stream << file.parts[i];
  }
  stream << " ]\n}";
  return stream;
}
std::ostream &operator<<(std::ostream &stream, const LogParts &parts) {
  stream << " {\n";
  if (!parts.log_event_uuid.empty()) {
    stream << "  \"log_event_uuid\": \"" << parts.log_event_uuid << "\",\n";
  }
  if (!parts.parts_uuid.empty()) {
    stream << "  \"parts_uuid\": \"" << parts.parts_uuid << "\",\n";
  }
  if (!parts.node.empty()) {
    stream << "  \"node\": \"" << parts.node << "\",\n";
  }
  if (!parts.source_boot_uuid.empty()) {
    stream << "  \"source_boot_uuid\": \"" << parts.source_boot_uuid << "\",\n";
    stream << "  \"boot_count\": " << parts.boot_count << ",\n";
  }
  stream << "  \"config\": \"" << parts.config.get() << "\"";
  if (!parts.config_sha256.empty()) {
    stream << ",\n  \"config_sha256\": \"" << parts.config_sha256 << "\"";
  }
  if (parts.logger_monotonic_start_time != aos::monotonic_clock::min_time) {
    stream << ",\n  \"logger_monotonic_start_time\": \""
           << parts.logger_monotonic_start_time << "\"";
  }
  if (parts.logger_realtime_start_time != aos::realtime_clock::min_time) {
    stream << ",\n  \"logger_realtime_start_time\": \""
           << parts.logger_realtime_start_time << "\"";
  }
  stream << ",\n  \"monotonic_start_time\": \"" << parts.monotonic_start_time
         << "\",\n  \"realtime_start_time\": \"" << parts.realtime_start_time
         << "\",\n  \"parts\": [";

  for (size_t i = 0; i < parts.parts.size(); ++i) {
    if (i != 0u) {
      stream << ", ";
    }
    stream << "\"" << parts.parts[i] << "\"";
  }

  stream << "]\n }";
  return stream;
}

std::string Sha256(const absl::Span<const uint8_t> str) {
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256_CTX sha256;
  SHA256_Init(&sha256);
  SHA256_Update(&sha256, str.data(), str.size());
  SHA256_Final(hash, &sha256);
  std::stringstream ss;
  for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
    ss << std::hex << std::setw(2) << std::setfill('0')
       << static_cast<int>(hash[i]);
  }
  return ss.str();
}

}  // namespace logger
}  // namespace aos
