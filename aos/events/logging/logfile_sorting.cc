#include "aos/events/logging/logfile_sorting.h"

#include <dirent.h>

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/btree_map.h"
#include "absl/strings/str_join.h"

#include "aos/events/logging/file_operations.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/flatbuffer_merge.h"
#include "aos/flatbuffers.h"
#include "aos/sha256.h"
#include "aos/time/time.h"
#include "sys/stat.h"

#if ENABLE_S3
#include "aos/events/logging/s3_file_operations.h"
#endif

DEFINE_bool(quiet_sorting, false,
            "If true, sort with minimal messages about truncated files.");

namespace aos {
namespace logger {
namespace {
namespace chrono = std::chrono;

std::unique_ptr<internal::FileOperations> MakeFileOperations(
    std::string_view filename) {
  static constexpr std::string_view kS3 = "s3:";
  if (filename.substr(0, kS3.size()) == kS3) {
#if ENABLE_S3
    return std::make_unique<internal::S3FileOperations>(filename);
#else
    LOG(FATAL) << "Reading files from S3 not supported on this platform";
#endif
  }
  if (filename.find("://") != filename.npos) {
    LOG(FATAL) << "This looks like a URL of an unknown type: " << filename;
  }
  return std::make_unique<internal::LocalFileOperations>(filename);
}

bool ConfigOnly(const LogFileHeader *header) {
  CHECK_EQ(LogFileHeader::MiniReflectTypeTable()->num_elems, 34u);
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
  if (header->has_boot_uuids()) return false;
  if (header->has_logger_part_monotonic_start_time()) return false;
  if (header->has_logger_part_realtime_start_time()) return false;
  if (header->has_oldest_remote_monotonic_timestamps()) return false;
  if (header->has_oldest_local_monotonic_timestamps()) return false;
  if (header->has_oldest_remote_unreliable_monotonic_timestamps()) return false;
  if (header->has_oldest_local_unreliable_monotonic_timestamps()) return false;
  if (header->has_oldest_remote_reliable_monotonic_timestamps()) return false;
  if (header->has_oldest_local_reliable_monotonic_timestamps()) return false;
  if (header->has_oldest_logger_remote_unreliable_monotonic_timestamps())
    return false;
  if (header->has_oldest_logger_local_unreliable_monotonic_timestamps())
    return false;
  if (header->has_logger_sha1()) return false;
  if (header->has_logger_version()) return false;

  return header->has_configuration();
}

bool HasNewTimestamps(const LogFileHeader *header) {
  if (header->has_oldest_remote_monotonic_timestamps()) {
    CHECK(header->has_oldest_local_monotonic_timestamps());
    CHECK(header->has_oldest_remote_unreliable_monotonic_timestamps());
    CHECK(header->has_oldest_local_unreliable_monotonic_timestamps());
    return true;
  } else {
    CHECK(!header->has_oldest_local_monotonic_timestamps());
    CHECK(!header->has_oldest_remote_unreliable_monotonic_timestamps());
    CHECK(!header->has_oldest_local_unreliable_monotonic_timestamps());
    return false;
  }
}

std::string ConcatenateParts(
    const std::vector<std::pair<std::string, int>> &parts) {
  std::vector<std::string_view> stringview_parts;
  stringview_parts.reserve(parts.size());
  for (const std::pair<std::string, int> &p : parts) {
    stringview_parts.emplace_back(p.first);
  }
  return absl::StrCat("[\"", absl::StrJoin(stringview_parts, "\", \""), "\"]");
}

}  // namespace

void FindLogs(std::vector<std::string> *files, std::string filename) {
  MakeFileOperations(filename)->FindLogs(files);
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
    const auto file_operations = MakeFileOperations(filename);
    if (file_operations->Exists()) {
      file_operations->FindLogs(&found_logfiles);
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
  // The logger instance UUID, all parts generated by a single Logger instance
  // will have the same value here.
  std::string logger_instance_uuid;
  // All log events across all nodes produced by a single high-level start event
  // will have the same value here.
  std::string log_start_uuid;

  // Pairs of the filename and the part index for sorting.
  std::vector<std::pair<std::string, int>> parts;

  std::string config_sha256;
};

// Struct to hold both the node, and the parts associated with it.
struct UnsortedLogPartsMap {
  std::string logger_node;
  // The boot UUID of the node this log file was created on.
  std::string logger_boot_uuid;
  // The logger instance UUID, all parts generated by a single Logger instance
  // will have the same value here.
  std::string logger_instance_uuid;
  // All log events across all nodes produced by a single high-level start event
  // will have the same value here.
  std::string log_start_uuid;

  aos::monotonic_clock::time_point monotonic_start_time =
      aos::monotonic_clock::min_time;
  aos::realtime_clock::time_point realtime_start_time =
      aos::realtime_clock::min_time;

  // Name from a log.  All logs below have been confirmed to match.
  std::string name;

  // Logger version info from the log.
  std::string logger_sha1;
  std::string logger_version;

  // Mapping from parts_uuid, source_boot_uuid -> parts.  We have log files
  // where the parts_uuid stays constant across reboots.
  std::map<std::pair<std::string, std::string>, UnsortedLogParts>
      unsorted_parts;
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

// This is an intermediate representation of Boots.  The data we have when we
// are iterating through the log file headers isn't enough to create Boots as we
// go, so we need to save an intermediate and then rewrite it into the final
// result once we know everything.
struct MapBoots {
  // Maps the boot UUID to the boot count.  Since boot UUIDs are unique, we
  // don't need to be node specific and can do this for all nodes.
  std::map<std::string, int> boot_count_map;

  // Maps the node name to a set of all boots for that node.
  std::map<std::string, std::vector<std::string>> boots;
};

// For a pair of nodes, this holds the oldest times that messages were
// transfered.
//
// TODO(austin): We are finding that the timestamps in
// oldest_remote_monotonic_timestamp are not very useful because there may not
// be reliable messages on both boots in the log.  Figure out how to use that
// data better (or really, use the new reliable remote timestamps field) and
// update this code.  I think we can delay a bit until someone figures out how
// to get here without other code paths sorting us first.
struct BootPairTimes {
  // Pair of local and remote timestamps for the oldest message forwarded to
  // this node.
  monotonic_clock::time_point oldest_remote_reliable_monotonic_timestamp;
  monotonic_clock::time_point oldest_local_reliable_monotonic_timestamp;

  // Pair of local and remote timestamps for the oldest unreliable message
  // forwarded to this node.
  monotonic_clock::time_point oldest_remote_unreliable_monotonic_timestamp;
  monotonic_clock::time_point oldest_local_unreliable_monotonic_timestamp;
};

std::ostream &operator<<(std::ostream &out, const BootPairTimes &time) {
  out << "{.oldest_remote_reliable_monotonic_timestamp="
      << time.oldest_remote_reliable_monotonic_timestamp
      << ", .oldest_local_reliable_monotonic_timestamp="
      << time.oldest_local_reliable_monotonic_timestamp
      << ", .oldest_remote_unreliable_monotonic_timestamp="
      << time.oldest_remote_unreliable_monotonic_timestamp
      << ", .oldest_local_unreliable_monotonic_timestamp="
      << time.oldest_local_unreliable_monotonic_timestamp << "}";
  return out;
}

aos::monotonic_clock::time_point MinLocalBootTime(const BootPairTimes &t) {
  return std::min(t.oldest_local_unreliable_monotonic_timestamp,
                  t.oldest_local_reliable_monotonic_timestamp);
}

aos::monotonic_clock::time_point MaxLocalBootTime(const BootPairTimes &t) {
  if (t.oldest_local_unreliable_monotonic_timestamp !=
      aos::monotonic_clock::max_time) {
    if (t.oldest_local_reliable_monotonic_timestamp !=
        aos::monotonic_clock::max_time) {
      return std::max(t.oldest_local_unreliable_monotonic_timestamp,
                      t.oldest_local_reliable_monotonic_timestamp);
    } else {
      return t.oldest_local_unreliable_monotonic_timestamp;
    }
  } else {
    return t.oldest_local_reliable_monotonic_timestamp;
  }
}

// Helper class to make it easier to sort a list of log files into
// std::vector<LogFile>
struct PartsSorter {
  // List of files that were corrupted.
  std::vector<std::string> corrupted;

  // Map from sha256 to the config.
  std::map<std::string, std::shared_ptr<const Configuration>>
      config_sha256_lookup;

  // List of all config sha256's we have found.
  std::set<std::string> config_sha256_list;

  // Map from an observed pair of boots to the associated timestamps.
  // local_node -> local_node_boot_uuid -> remote_node index ->
  // remote_boot_uuid -> list of all times from all parts.
  absl::btree_map<
      std::string,
      absl::btree_map<
          std::string,
          absl::btree_map<size_t, absl::btree_map<std::string,
                                                  std::vector<BootPairTimes>>>>>
      boot_times;

  // Map holding the log_event_uuid -> second map.  The second map holds the
  // parts_uuid -> list of parts for sorting.
  std::map<std::string, UnsortedLogPartsMap> parts_list;

  // A list of all the old parts which we don't know how to sort using uuids.
  // There are enough of these in the wild that this is worth supporting.
  std::vector<UnsortedOldParts> old_parts;

  // Populates the class's datastructures from the input file list.
  void PopulateFromFiles(ReadersPool *readers,
                         const std::vector<std::string> &parts);

  // Wrangles everything into a map of boot uuids -> boot counts.
  MapBoots ComputeBootCounts();

  // Computes the boot constraints to solve for old and new logs.
  std::map<std::string, NodeBootState> ComputeOldBootConstraints();
  std::map<std::string, NodeBootState> ComputeNewBootConstraints();

  // Reformats old_parts into a list of logfiles and returns it.  This destroys
  // state in PartsSorter.
  std::vector<LogFile> FormatOldParts();

  // Reformats parts_list into a list of logfiles and returns it.  This destroys
  // state in PartsSorter.
  std::vector<LogFile> FormatNewParts();

  // Returns a list of all the parts that we have sorted.
  std::vector<LogFile> SortParts();
};

void PartsSorter::PopulateFromFiles(ReadersPool *readers,
                                    const std::vector<std::string> &parts) {
  // Now extract everything into our datastructures above for sorting.
  for (const std::string &part : parts) {
    SpanReader *reader = readers->BorrowReader(part);
    std::optional<SizePrefixedFlatbufferVector<LogFileHeader>> log_header =
        ReadHeader(reader);
    if (!log_header) {
      if (!FLAGS_quiet_sorting) {
        LOG(WARNING) << "Skipping " << part << " without a header";
      }
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

    const std::string_view logger_sha1 =
        log_header->message().has_logger_sha1()
            ? log_header->message().logger_sha1()->string_view()
            : "";

    const std::string_view logger_version =
        log_header->message().has_logger_version()
            ? log_header->message().logger_version()->string_view()
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

    const std::string_view logger_instance_uuid =
        log_header->message().has_logger_instance_uuid()
            ? log_header->message().logger_instance_uuid()->string_view()
            : "";

    const std::string_view log_start_uuid =
        log_header->message().has_log_start_uuid()
            ? log_header->message().log_start_uuid()->string_view()
            : "";

    std::string configuration_sha256 =
        log_header->message().has_configuration_sha256()
            ? log_header->message().configuration_sha256()->str()
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
      config_sha256_list.emplace(hash);
      continue;
    }

    VLOG(1) << "Header " << FlatbufferToJson(log_header.value()) << " " << part;

    if (configuration_sha256.empty()) {
      CHECK(log_header->message().has_configuration())
          << ": Failed to find header on " << part;
      // If we don't have a configuration_sha256, we need to have a
      // configuration directly inside the header.  This ends up being a bit
      // unwieldy to deal with, so let's instead copy the configuration, hash
      // it, and then use the configuration_sha256 that we found to continue
      // down the old code path.

      // Do a recursive copy to normalize the flatbuffer.  Different
      // configurations can be built different ways, and can even have their
      // vtable out of order.  Don't think and just trigger a copy.
      const FlatbufferDetachedBuffer<Configuration> config_copy =
          RecursiveCopyFlatBuffer(log_header->message().configuration());
      std::string config_copy_sha256 = Sha256(config_copy.span());

      if (config_sha256_lookup.find(config_copy_sha256) ==
          config_sha256_lookup.end()) {
        auto header =
            std::make_shared<SizePrefixedFlatbufferVector<LogFileHeader>>(
                *log_header);
        config_sha256_lookup.emplace(
            config_copy_sha256, std::shared_ptr<const Configuration>(
                                    header, header->message().configuration()));
      }
      config_sha256_list.emplace(config_copy_sha256);
      configuration_sha256 = std::move(config_copy_sha256);
    } else {
      CHECK(!log_header->message().has_configuration())
          << ": Found header where one shouldn't be on " << part;
      config_sha256_list.emplace(configuration_sha256);
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
        if (!FLAGS_quiet_sorting) {
          LOG(WARNING) << "Skipping " << part << " without any messages";
        }
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
        old_parts.back().parts.config_sha256 = configuration_sha256;
        old_parts.back().unsorted_parts.emplace_back(
            std::make_pair(first_message_time, part));
        old_parts.back().name = name;
      } else {
        result->unsorted_parts.emplace_back(
            std::make_pair(first_message_time, part));
        CHECK_EQ(result->name, name);
        CHECK_EQ(result->parts.config_sha256, configuration_sha256);
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
      log_it->second.log_start_uuid = log_start_uuid;
      log_it->second.logger_instance_uuid = logger_instance_uuid;
      log_it->second.name = name;
      log_it->second.logger_sha1 = logger_sha1;
      log_it->second.logger_version = logger_version;
    } else {
      CHECK_EQ(log_it->second.logger_node, logger_node);
      CHECK_EQ(log_it->second.logger_boot_uuid, logger_boot_uuid);
      CHECK_EQ(log_it->second.log_start_uuid, log_start_uuid);
      CHECK_EQ(log_it->second.logger_instance_uuid, logger_instance_uuid);
      CHECK_EQ(log_it->second.name, name);
      CHECK_EQ(log_it->second.logger_sha1, logger_sha1);
      CHECK_EQ(log_it->second.logger_version, logger_version);
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

    VLOG(1) << "Parts: " << parts_uuid << ", source boot uuid "
            << source_boot_uuid << " index " << parts_index;
    auto it = log_it->second.unsorted_parts.find(
        std::pair(parts_uuid, std::string(source_boot_uuid)));
    if (it == log_it->second.unsorted_parts.end()) {
      it = log_it->second.unsorted_parts
               .insert(std::make_pair(
                   std::make_pair(parts_uuid, std::string(source_boot_uuid)),
                   UnsortedLogParts()))
               .first;
      it->second.monotonic_start_time = monotonic_start_time;
      it->second.realtime_start_time = realtime_start_time;
      it->second.logger_monotonic_start_time = logger_monotonic_start_time;
      it->second.logger_realtime_start_time = logger_realtime_start_time;
      it->second.node = std::string(node);
      it->second.source_boot_uuid = source_boot_uuid;
      it->second.logger_instance_uuid = logger_instance_uuid;
      it->second.log_start_uuid = log_start_uuid;
      it->second.config_sha256 = configuration_sha256;
    } else {
      CHECK_EQ(it->second.config_sha256, configuration_sha256);
    }

    // We've got a newer log with boot_uuids, and oldest timestamps.  Fill in
    // this->boot_times with the info we have found.
    if (HasNewTimestamps(&log_header->message())) {
      auto node_boot_times_it = boot_times.find(node);
      if (node_boot_times_it == boot_times.end()) {
        node_boot_times_it =
            boot_times
                .emplace(
                    node,
                    absl::btree_map<
                        std::string,
                        absl::btree_map<
                            size_t,
                            absl::btree_map<std::string,
                                            std::vector<BootPairTimes>>>>())
                .first;
      }

      auto source_boot_times_it =
          node_boot_times_it->second.find(std::string(source_boot_uuid));

      if (source_boot_times_it == node_boot_times_it->second.end()) {
        source_boot_times_it =
            node_boot_times_it->second
                .emplace(
                    source_boot_uuid,
                    absl::btree_map<
                        size_t, absl::btree_map<std::string,
                                                std::vector<BootPairTimes>>>())
                .first;
      }

      // Older logs don't have our fancy new boot_uuids and oldest timestamps.
      // So only fill those out when we find them.
      CHECK(log_header->message().has_boot_uuids());
      const size_t boot_uuids_size = log_header->message().boot_uuids()->size();
      CHECK_EQ(
          boot_uuids_size,
          log_header->message().oldest_local_monotonic_timestamps()->size());
      CHECK_EQ(
          boot_uuids_size,
          log_header->message().oldest_remote_monotonic_timestamps()->size());
      CHECK_EQ(boot_uuids_size,
               log_header->message()
                   .oldest_local_unreliable_monotonic_timestamps()
                   ->size());
      CHECK_EQ(boot_uuids_size,
               log_header->message()
                   .oldest_remote_unreliable_monotonic_timestamps()
                   ->size());
      CHECK_EQ(log_header->message()
                   .has_oldest_logger_local_unreliable_monotonic_timestamps(),
               log_header->message()
                   .has_oldest_logger_remote_unreliable_monotonic_timestamps());
      if (log_header->message()
              .has_oldest_logger_local_unreliable_monotonic_timestamps()) {
        CHECK_EQ(boot_uuids_size,
                 log_header->message()
                     .oldest_logger_local_unreliable_monotonic_timestamps()
                     ->size());
        CHECK_EQ(boot_uuids_size,
                 log_header->message()
                     .oldest_logger_remote_unreliable_monotonic_timestamps()
                     ->size());
      }
      CHECK(!logger_boot_uuid.empty());
      CHECK(!source_boot_uuid.empty());
      for (size_t node_index = 0; node_index < boot_uuids_size; ++node_index) {
        const std::string_view boot_uuid =
            log_header->message().boot_uuids()->Get(node_index)->string_view();

        const monotonic_clock::time_point oldest_local_monotonic_timestamp(
            chrono::nanoseconds(
                log_header->message().oldest_local_monotonic_timestamps()->Get(
                    node_index)));
        const monotonic_clock::time_point oldest_remote_monotonic_timestamp(
            chrono::nanoseconds(
                log_header->message().oldest_remote_monotonic_timestamps()->Get(
                    node_index)));
        const monotonic_clock::time_point
            oldest_local_unreliable_monotonic_timestamp(chrono::nanoseconds(
                log_header->message()
                    .oldest_local_unreliable_monotonic_timestamps()
                    ->Get(node_index)));
        const monotonic_clock::time_point
            oldest_remote_unreliable_monotonic_timestamp(chrono::nanoseconds(
                log_header->message()
                    .oldest_remote_unreliable_monotonic_timestamps()
                    ->Get(node_index)));

        const monotonic_clock::time_point
            oldest_logger_local_unreliable_monotonic_timestamp =
                log_header->message()
                        .has_oldest_logger_local_unreliable_monotonic_timestamps()
                    ? monotonic_clock::time_point(chrono::nanoseconds(
                          log_header->message()
                              .oldest_logger_local_unreliable_monotonic_timestamps()
                              ->Get(node_index)))
                    : monotonic_clock::max_time;
        const monotonic_clock::time_point
            oldest_logger_remote_unreliable_monotonic_timestamp =
                log_header->message()
                        .has_oldest_logger_remote_unreliable_monotonic_timestamps()
                    ? monotonic_clock::time_point(chrono::nanoseconds(
                          log_header->message()
                              .oldest_logger_remote_unreliable_monotonic_timestamps()
                              ->Get(node_index)))
                    : monotonic_clock::max_time;

        const monotonic_clock::time_point
            oldest_local_reliable_monotonic_timestamp =
                log_header->message()
                        .has_oldest_local_reliable_monotonic_timestamps()
                    ? monotonic_clock::time_point(chrono::nanoseconds(
                          log_header->message()
                              .oldest_local_reliable_monotonic_timestamps()
                              ->Get(node_index)))
                    : monotonic_clock::max_time;
        const monotonic_clock::time_point
            oldest_remote_reliable_monotonic_timestamp =
                log_header->message()
                        .has_oldest_remote_reliable_monotonic_timestamps()
                    ? monotonic_clock::time_point(chrono::nanoseconds(
                          log_header->message()
                              .oldest_remote_reliable_monotonic_timestamps()
                              ->Get(node_index)))
                    : monotonic_clock::max_time;
        if (boot_uuid.empty()) {
          CHECK_EQ(oldest_local_monotonic_timestamp, monotonic_clock::max_time);
          CHECK_EQ(oldest_remote_monotonic_timestamp,
                   monotonic_clock::max_time);
          CHECK_EQ(oldest_local_unreliable_monotonic_timestamp,
                   monotonic_clock::max_time);
          CHECK_EQ(oldest_remote_unreliable_monotonic_timestamp,
                   monotonic_clock::max_time);
          CHECK_EQ(oldest_logger_local_unreliable_monotonic_timestamp,
                   monotonic_clock::max_time);
          CHECK_EQ(oldest_logger_remote_unreliable_monotonic_timestamp,
                   monotonic_clock::max_time);
          CHECK_EQ(oldest_local_reliable_monotonic_timestamp,
                   monotonic_clock::max_time);
          CHECK_EQ(oldest_remote_reliable_monotonic_timestamp,
                   monotonic_clock::max_time);
          continue;
        }

        if (boot_uuid == source_boot_uuid) {
          CHECK_EQ(oldest_local_monotonic_timestamp, monotonic_clock::max_time);
          CHECK_EQ(oldest_remote_monotonic_timestamp,
                   monotonic_clock::max_time);
          CHECK_EQ(oldest_local_unreliable_monotonic_timestamp,
                   monotonic_clock::max_time);
          CHECK_EQ(oldest_remote_unreliable_monotonic_timestamp,
                   monotonic_clock::max_time);
          CHECK_EQ(oldest_local_reliable_monotonic_timestamp,
                   monotonic_clock::max_time);
          CHECK_EQ(oldest_remote_reliable_monotonic_timestamp,
                   monotonic_clock::max_time);
          if (oldest_logger_local_unreliable_monotonic_timestamp !=
              monotonic_clock::max_time) {
            CHECK_NE(oldest_logger_remote_unreliable_monotonic_timestamp,
                     monotonic_clock::max_time);
            // Now, we found a timestamp going the other way.  Add it in!
            auto logger_node_boot_times_it = boot_times.find(logger_node);
            if (logger_node_boot_times_it == boot_times.end()) {
              logger_node_boot_times_it =
                  boot_times
                      .emplace(
                          logger_node,
                          absl::btree_map<
                              std::string,
                              absl::btree_map<
                                  size_t, absl::btree_map<
                                              std::string,
                                              std::vector<BootPairTimes>>>>())
                      .first;
            }

            auto logger_source_boot_times_it =
                logger_node_boot_times_it->second.find(
                    std::string(logger_boot_uuid));

            if (logger_source_boot_times_it ==
                logger_node_boot_times_it->second.end()) {
              logger_source_boot_times_it =
                  logger_node_boot_times_it->second
                      .emplace(
                          logger_boot_uuid,
                          absl::btree_map<
                              size_t,
                              absl::btree_map<std::string,
                                              std::vector<BootPairTimes>>>())
                      .first;
            }

            // We need the index of the source node.  Luckily, since we are at
            // the index in the boot UUID list which matches the source node
            // boot uuid, we know it's index!
            auto logger_destination_boot_times_it =
                logger_source_boot_times_it->second.find(node_index);
            if (logger_destination_boot_times_it ==
                logger_source_boot_times_it->second.end()) {
              logger_destination_boot_times_it =
                  logger_source_boot_times_it->second
                      .emplace(node_index,
                               absl::btree_map<std::string,
                                               std::vector<BootPairTimes>>())
                      .first;
            }

            auto logger_boot_times_it =
                logger_destination_boot_times_it->second.find(
                    std::string(source_boot_uuid));

            if (logger_boot_times_it ==
                logger_destination_boot_times_it->second.end()) {
              // We have a new boot UUID pairing.  Copy over the data we have.
              logger_destination_boot_times_it->second.emplace(
                  source_boot_uuid,
                  std::vector<BootPairTimes>{BootPairTimes{
                      .oldest_remote_reliable_monotonic_timestamp =
                          monotonic_clock::max_time,
                      .oldest_local_reliable_monotonic_timestamp =
                          monotonic_clock::max_time,
                      .oldest_remote_unreliable_monotonic_timestamp =
                          oldest_logger_remote_unreliable_monotonic_timestamp,
                      .oldest_local_unreliable_monotonic_timestamp =
                          oldest_logger_local_unreliable_monotonic_timestamp}});
            } else {
              logger_boot_times_it->second.emplace_back(BootPairTimes{
                  .oldest_remote_reliable_monotonic_timestamp =
                      oldest_remote_reliable_monotonic_timestamp,
                  .oldest_local_reliable_monotonic_timestamp =
                      oldest_local_reliable_monotonic_timestamp,
                  .oldest_remote_unreliable_monotonic_timestamp =
                      oldest_logger_remote_unreliable_monotonic_timestamp,
                  .oldest_local_unreliable_monotonic_timestamp =
                      oldest_logger_local_unreliable_monotonic_timestamp});
            }
          }
          continue;
        }

        // There is no supported way to get logger timestamps from anything but
        // the source node.  Since we've already handled that above, we should
        // always expect max_time here.
        CHECK_EQ(oldest_logger_local_unreliable_monotonic_timestamp,
                 monotonic_clock::max_time);
        CHECK_EQ(oldest_logger_remote_unreliable_monotonic_timestamp,
                 monotonic_clock::max_time);

        // Now, we have a valid pairing.
        auto destination_boot_times_it =
            source_boot_times_it->second.find(node_index);
        if (destination_boot_times_it == source_boot_times_it->second.end()) {
          destination_boot_times_it =
              source_boot_times_it->second
                  .emplace(node_index,
                           absl::btree_map<std::string,
                                           std::vector<BootPairTimes>>())
                  .first;
        }

        auto boot_times_it =
            destination_boot_times_it->second.find(std::string(boot_uuid));

        if (boot_times_it == destination_boot_times_it->second.end()) {
          // We have a new boot UUID pairing.  Copy over the data we have.
          destination_boot_times_it->second.emplace(
              boot_uuid, std::vector<BootPairTimes>{BootPairTimes{
                             .oldest_remote_reliable_monotonic_timestamp =
                                 oldest_remote_reliable_monotonic_timestamp,
                             .oldest_local_reliable_monotonic_timestamp =
                                 oldest_local_reliable_monotonic_timestamp,
                             .oldest_remote_unreliable_monotonic_timestamp =
                                 oldest_remote_unreliable_monotonic_timestamp,
                             .oldest_local_unreliable_monotonic_timestamp =
                                 oldest_local_unreliable_monotonic_timestamp}});
        } else {
          boot_times_it->second.emplace_back(
              BootPairTimes{.oldest_remote_reliable_monotonic_timestamp =
                                oldest_remote_reliable_monotonic_timestamp,
                            .oldest_local_reliable_monotonic_timestamp =
                                oldest_local_reliable_monotonic_timestamp,
                            .oldest_remote_unreliable_monotonic_timestamp =
                                oldest_remote_unreliable_monotonic_timestamp,
                            .oldest_local_unreliable_monotonic_timestamp =
                                oldest_local_unreliable_monotonic_timestamp});
        }
      }
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

    auto it = config_sha256_lookup.find(p.parts.config_sha256);
    CHECK(it != config_sha256_lookup.end());

    log_file.config = it->second;
    log_file.config_sha256 = p.parts.config_sha256;

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

std::map<std::string, NodeBootState> PartsSorter::ComputeNewBootConstraints() {
  std::map<std::string, NodeBootState> boot_constraints;

  CHECK_EQ(config_sha256_list.size(), 1u) << ": Found more than one config";
  auto config_it = config_sha256_lookup.find(*config_sha256_list.begin());
  CHECK(config_it != config_sha256_lookup.end())
      << ": Failed to find a config with a sha256 of "
      << *config_sha256_list.begin();
  const Configuration *config = config_it->second.get();

  // Map from an observed pair of boots to the associated timestamps.
  // remote_node -> remote_node_boot_uuid -> local_node ->
  // local_boot_uuid -> list of all times from all parts.
  //
  // This is boot_times but flipped inside out so we can order on remote
  // instead of local for when we have the same remote for 2 different local
  // boots.
  absl::btree_map<
      std::string,
      absl::btree_map<
          std::string,
          absl::btree_map<std::string,
                          absl::btree_map<std::string, BootPairTimes>>>>
      reverse_boot_times;

  for (const auto &local_node_boot_times : boot_times) {
    const std::string &local_node_name = local_node_boot_times.first;

    // We know nothing about the order of the local node's boot, but we
    // know it happened.  If there is only 1 single boot, the constraint
    // code will happily mark it as boot 0.  Otherwise we'll get the
    // appropriate boot count if it can be computed or an error.
    //
    // Add it to the boots list to kick this off.
    auto logger_node_boot_constraints_it =
        boot_constraints.find(local_node_name);
    if (logger_node_boot_constraints_it == boot_constraints.end()) {
      logger_node_boot_constraints_it =
          boot_constraints
              .insert(std::make_pair(local_node_name, NodeBootState()))
              .first;
    }

    for (const auto &local_boot_time : local_node_boot_times.second) {
      const std::string &local_boot_uuid = local_boot_time.first;
      logger_node_boot_constraints_it->second.boots.insert(local_boot_uuid);

      for (const auto &remote_node : local_boot_time.second) {
        const std::string remote_node_name =
            config->nodes()->Get(remote_node.first)->name()->str();

        VLOG(1) << "Local " << local_node_name << " boot " << local_boot_uuid
                << " remote " << remote_node_name;

        // Now, we have a bunch of remote boots for the same local boot and
        // remote node.  We want to sort them by observed local time.  This will
        // tell us which ones happened first.  Hold on to the max time on that
        // node too so we can check for overlapping boots.
        std::vector<std::tuple<std::string, BootPairTimes, BootPairTimes>>
            remote_boot_times;
        for (const auto &boot_time_list : remote_node.second) {
          // Track the first boot time we have evidence of.
          BootPairTimes boot_time = boot_time_list.second[0];
          // And the last one so we can look for overlapping boots.
          BootPairTimes max_boot_time = boot_time_list.second[0];
          for (size_t i = 0; i < boot_time_list.second.size(); ++i) {
            const BootPairTimes &next_boot_time = boot_time_list.second[i];
            VLOG(1) << " Found " << next_boot_time;
            if (next_boot_time.oldest_local_unreliable_monotonic_timestamp !=
                aos::monotonic_clock::max_time) {
              VLOG(1)
                  << "  Unreliable remote time "
                  << next_boot_time.oldest_remote_unreliable_monotonic_timestamp
                  << " remote " << boot_time_list.first << " -> local time "
                  << next_boot_time.oldest_local_unreliable_monotonic_timestamp
                  << " local " << local_boot_uuid;
            }
            if (next_boot_time.oldest_local_reliable_monotonic_timestamp !=
                aos::monotonic_clock::max_time) {
              VLOG(1)
                  << "  Reliable remote time "
                  << next_boot_time.oldest_remote_reliable_monotonic_timestamp
                  << " remote " << boot_time_list.first << " -> local time "
                  << next_boot_time.oldest_local_reliable_monotonic_timestamp
                  << " local " << local_boot_uuid;
            }
            // If we found an existing entry, update the min to be the min of
            // all records.  This lets us combine info from multiple part files.
            if (next_boot_time.oldest_remote_reliable_monotonic_timestamp <
                boot_time.oldest_remote_reliable_monotonic_timestamp) {
              boot_time.oldest_remote_reliable_monotonic_timestamp =
                  next_boot_time.oldest_remote_reliable_monotonic_timestamp;
              boot_time.oldest_local_reliable_monotonic_timestamp =
                  next_boot_time.oldest_local_reliable_monotonic_timestamp;
            }

            if ((next_boot_time.oldest_remote_reliable_monotonic_timestamp >
                     max_boot_time.oldest_remote_reliable_monotonic_timestamp ||
                 max_boot_time.oldest_remote_reliable_monotonic_timestamp ==
                     aos::monotonic_clock::max_time) &&
                next_boot_time.oldest_remote_reliable_monotonic_timestamp !=
                    aos::monotonic_clock::max_time) {
              max_boot_time.oldest_remote_reliable_monotonic_timestamp =
                  next_boot_time.oldest_remote_reliable_monotonic_timestamp;
              max_boot_time.oldest_local_reliable_monotonic_timestamp =
                  next_boot_time.oldest_local_reliable_monotonic_timestamp;
            }

            if (next_boot_time.oldest_remote_unreliable_monotonic_timestamp <
                boot_time.oldest_remote_unreliable_monotonic_timestamp) {
              boot_time.oldest_remote_unreliable_monotonic_timestamp =
                  next_boot_time.oldest_remote_unreliable_monotonic_timestamp;
              boot_time.oldest_local_unreliable_monotonic_timestamp =
                  next_boot_time.oldest_local_unreliable_monotonic_timestamp;
            }
            if ((next_boot_time.oldest_remote_unreliable_monotonic_timestamp >
                     max_boot_time
                         .oldest_remote_unreliable_monotonic_timestamp ||
                 max_boot_time.oldest_remote_unreliable_monotonic_timestamp ==
                     aos::monotonic_clock::max_time) &&
                next_boot_time.oldest_remote_unreliable_monotonic_timestamp !=
                    aos::monotonic_clock::max_time) {
              max_boot_time.oldest_remote_unreliable_monotonic_timestamp =
                  next_boot_time.oldest_remote_unreliable_monotonic_timestamp;
              max_boot_time.oldest_local_unreliable_monotonic_timestamp =
                  next_boot_time.oldest_local_unreliable_monotonic_timestamp;
            }
          }

          // Skip anything without a time in it.
          if (boot_time.oldest_remote_unreliable_monotonic_timestamp ==
                  aos::monotonic_clock::max_time &&
              boot_time.oldest_remote_reliable_monotonic_timestamp ==
                  aos::monotonic_clock::max_time) {
            VLOG(1) << " Skipping local " << local_node_name << " boot "
                    << local_boot_uuid << " remote " << remote_node_name
                    << " boot " << boot_time_list.first << " " << boot_time;
            continue;
          }

          remote_boot_times.emplace_back(
              std::make_tuple(boot_time_list.first, boot_time, max_boot_time));

          // While we are building up the forwards set of constraints, build up
          // a list of reverse constraints.  This gives us a list of boots for
          // the case when we have 2 logs from different boots, where the remote
          // is both the same boot.

          auto reverse_remote_node_it =
              reverse_boot_times.find(remote_node_name);
          if (reverse_remote_node_it == reverse_boot_times.end()) {
            reverse_remote_node_it =
                reverse_boot_times
                    .emplace(
                        remote_node_name,
                        absl::btree_map<
                            std::string,
                            absl::btree_map<
                                std::string,
                                absl::btree_map<std::string, BootPairTimes>>>())
                    .first;
          }

          auto reverse_remote_node_boot_uuid_it =
              reverse_remote_node_it->second.find(boot_time_list.first);
          if (reverse_remote_node_boot_uuid_it ==
              reverse_remote_node_it->second.end()) {
            reverse_remote_node_boot_uuid_it =
                reverse_remote_node_it->second
                    .emplace(boot_time_list.first,
                             absl::btree_map<
                                 std::string,
                                 absl::btree_map<std::string, BootPairTimes>>())
                    .first;
          }

          auto reverse_local_node_it =
              reverse_remote_node_boot_uuid_it->second.find(local_node_name);
          if (reverse_local_node_it ==
              reverse_remote_node_boot_uuid_it->second.end()) {
            reverse_local_node_it =
                reverse_remote_node_boot_uuid_it->second
                    .emplace(local_node_name,
                             absl::btree_map<std::string, BootPairTimes>())
                    .first;
          }

          auto reverse_local_node_boot_uuid_it =
              reverse_local_node_it->second.find(local_boot_uuid);
          CHECK(reverse_local_node_boot_uuid_it ==
                reverse_local_node_it->second.end());
          reverse_local_node_it->second.emplace(local_boot_uuid, boot_time);
          VLOG(1) << " Boot time for local " << local_node_name << " boot "
                  << local_boot_uuid << " remote " << remote_node_name
                  << " boot " << boot_time_list.first << " " << boot_time;
        }

        // All the nodes are from the same local boot here.  We are trying to
        // order the remote boots.  Let's list out the combinatorics.
        //  - We can have reliable and/or unreliable timestamps, but not neither
        //    for both times we are comparing.
        //  - Reliable timestamps can match or not match (ie, same message got
        //    forwarded to 2 boots)
        //
        //  1) Both have unreliable, various reliable.  1 < 2.
        //     Unreliable timestamps tell us more
        //      {
        //       .oldest_remote_reliable_monotonic_timestamp=10.122999611sec,
        //       .oldest_local_reliable_monotonic_timestamp=9.400951024sec,
        //       .oldest_remote_unreliable_monotonic_timestamp=23431.315344025sec,
        //       .oldest_local_unreliable_monotonic_timestamp=23413.172709284sec
        //      }
        //      {
        //       .oldest_remote_reliable_monotonic_timestamp=11.798054208sec,
        //       .oldest_local_reliable_monotonic_timestamp=23457.772660691sec,
        //       .oldest_remote_unreliable_monotonic_timestamp=12.315344025sec,
        //       .oldest_local_unreliable_monotonic_timestamp=23458.772660691sec
        //      }
        // 2) Only reliable, or only one unreliable, local times don't match.  1
        // < 2
        //      {
        //       .oldest_remote_reliable_monotonic_timestamp=10.122999611sec,
        //       .oldest_local_reliable_monotonic_timestamp=9.400951024sec,
        //       .oldest_remote_unreliable_monotonic_timestamp=9223372036.854775807sec,
        //       .oldest_local_unreliable_monotonic_timestamp=9223372036.854775807sec
        //      }
        //      {
        //       .oldest_remote_reliable_monotonic_timestamp=11.798054208sec,
        //       .oldest_local_reliable_monotonic_timestamp=23457.772660691sec,
        //       .oldest_remote_unreliable_monotonic_timestamp=9223372036.854775807sec,
        //       .oldest_local_unreliable_monotonic_timestamp=9223372036.854775807sec
        //      }
        //  3) Only reliable, local times match.  Unable to compare because the
        //     same message got sent, and with reliable timestamps, we don't
        //     know how long it took to cross the network.
        //      {
        //       .oldest_remote_reliable_monotonic_timestamp=10.122999611sec,
        //       .oldest_local_reliable_monotonic_timestamp=9.400951024sec,
        //       .oldest_remote_unreliable_monotonic_timestamp=9223372036.854775807sec,
        //       .oldest_local_unreliable_monotonic_timestamp=9223372036.854775807sec
        //      }
        //      {
        //       .oldest_remote_reliable_monotonic_timestamp=11.798054208sec,
        //       .oldest_local_reliable_monotonic_timestamp=9.400951024sec,
        //       .oldest_remote_unreliable_monotonic_timestamp=9223372036.854775807sec,
        //       .oldest_local_unreliable_monotonic_timestamp=9223372036.854775807sec
        //      }
        //  4) One reliable, one unreliable, local times don't match. 1 < 2
        //     same message got sent, and with reliable timestamps, we don't
        //     know how long it took to cross the network.
        //      {
        //       .oldest_remote_reliable_monotonic_timestamp=9223372036.854775807sec,
        //       .oldest_local_reliable_monotonic_timestamp=9223372036.854775807sec
        //       .oldest_remote_unreliable_monotonic_timestamp=10.122999611sec,
        //       .oldest_local_unreliable_monotonic_timestamp=9.400951024sec,
        //      }
        //      {
        //       .oldest_remote_reliable_monotonic_timestamp=11.798054208sec,
        //       .oldest_local_reliable_monotonic_timestamp=23457.772660691sec,
        //       .oldest_remote_unreliable_monotonic_timestamp=9223372036.854775807sec,
        //       .oldest_local_unreliable_monotonic_timestamp=9223372036.854775807sec
        //      }
        //
        //  Writing all this out for which timestamps we have out of all 32
        //  combinations, and which cases each of the correspond to:
        //
        //  {  }, {  } no match -> fail, won't be in the list
        //  {  }, { u} no match -> fail, won't be in the list
        //  {  }, {r } no match -> fail, won't be in the list
        //  {  }, {ru} no match -> fail, won't be in the list
        //  { u}, {  } no match -> fail, won't be in the list
        //  { u}, { u} no match -> 1
        //  { u}, {r } no match -> 4
        //  { u}, {ru} no match -> 1
        //  {r }, {  } no match -> fail, won't be in the list
        //  {r }, { u} no match -> 4
        //  {r }, {r } no match -> 2
        //  {r }, {ru} no match -> 2
        //  {ru}, {  } no match -> fail, won't be in the list
        //  {ru}, { u} no match -> 1
        //  {ru}, {r } no match -> 2
        //  {ru}, {ru} no match -> 1
        //  {  }, {  } match -> fail, won't be in the list
        //  {  }, { u} match -> fail, won't be in the list
        //  {  }, {r } match -> fail, won't be in the list
        //  {  }, {ru} match -> fail, won't be in the list
        //  { u}, {  } match -> fail, won't be in the list
        //  { u}, { u} match -> 1
        //  { u}, {r } match -> fail
        //  { u}, {ru} match -> 1
        //  {r }, {  } match -> fail, won't be in the list
        //  {r }, { u} match -> fail
        //  {r }, {r } match -> fail (case 3)
        //  {r }, {ru} match -> fail (case 3)
        //  {ru}, {  } match -> fail, won't be in the list
        //  {ru}, { u} match -> 1
        //  {ru}, {r } match -> fail (case 3)
        //  {ru}, {ru} match -> 1
        //
        //  Combined, we get:
        //
        //  { u}, { u} no match -> 1
        //  { u}, {ru} no match -> 1
        //  {ru}, { u} no match -> 1
        //  {ru}, {ru} no match -> 1
        //  { u}, { u} match -> 1
        //  { u}, {ru} match -> 1
        //  {ru}, { u} match -> 1
        //  {ru}, {ru} match -> 1
        //
        //  {r }, {r } no match -> 2
        //  {r }, {ru} no match -> 2
        //  {ru}, {r } no match -> 2
        //
        //  { u}, {r } no match -> fail
        //  { u}, {r } match -> fail
        //  {r }, { u} no match -> fail
        //  {r }, { u} match -> fail
        //
        //  {r }, {r } match -> fail (case 3)
        //  {r }, {ru} match -> fail (case 3)
        //  {ru}, {r } match -> fail (case 3)

        std::sort(
            remote_boot_times.begin(), remote_boot_times.end(),
            [](const std::tuple<std::string, BootPairTimes, BootPairTimes> &a,
               const std::tuple<std::string, BootPairTimes, BootPairTimes> &b) {
              const bool both_reliable =
                  std::get<1>(a).oldest_local_reliable_monotonic_timestamp !=
                      aos::monotonic_clock::max_time &&
                  std::get<1>(b).oldest_local_reliable_monotonic_timestamp !=
                      aos::monotonic_clock::max_time;
              const bool both_unreliable =
                  std::get<1>(a).oldest_local_unreliable_monotonic_timestamp !=
                      aos::monotonic_clock::max_time &&
                  std::get<1>(b).oldest_local_unreliable_monotonic_timestamp !=
                      aos::monotonic_clock::max_time;

              if (both_unreliable) {
                VLOG(1) << "Both Unreliable";
                return std::get<1>(a)
                           .oldest_local_unreliable_monotonic_timestamp <
                       std::get<1>(b)
                           .oldest_local_unreliable_monotonic_timestamp;
              } else if (both_reliable) {
                VLOG(1) << "Both Reliable";
                CHECK_NE(
                    std::get<1>(a).oldest_local_reliable_monotonic_timestamp,
                    std::get<1>(b).oldest_local_reliable_monotonic_timestamp)
                    << ": Broken logic, the same reliable message has been "
                       "forwarded to both boots.  This is ambiguous, please "
                       "investigate.";
                return std::get<1>(a)
                           .oldest_local_reliable_monotonic_timestamp <
                       std::get<1>(b).oldest_local_reliable_monotonic_timestamp;

              } else if (std::get<1>(a)
                                 .oldest_local_reliable_monotonic_timestamp !=
                             aos::monotonic_clock::max_time &&
                         std::get<1>(b)
                                 .oldest_local_unreliable_monotonic_timestamp !=
                             aos::monotonic_clock::max_time) {
                VLOG(1)
                    << " Comparing Reliable  "
                    << std::get<1>(a).oldest_local_reliable_monotonic_timestamp;
                VLOG(1) << "   Versus Uneliable  "
                        << std::get<1>(b)
                               .oldest_local_unreliable_monotonic_timestamp;

                return std::get<1>(a)
                           .oldest_local_reliable_monotonic_timestamp <
                       std::get<1>(b)
                           .oldest_local_unreliable_monotonic_timestamp;

              } else if (std::get<1>(a)
                                 .oldest_local_unreliable_monotonic_timestamp !=
                             aos::monotonic_clock::max_time &&
                         std::get<1>(b)
                                 .oldest_local_reliable_monotonic_timestamp !=
                             aos::monotonic_clock::max_time) {
                VLOG(1) << " Comparing Unreliable  "
                        << std::get<1>(a)
                               .oldest_local_unreliable_monotonic_timestamp;
                VLOG(1)
                    << "   Versus Reliable     "
                    << std::get<1>(b).oldest_local_reliable_monotonic_timestamp;

                return std::get<1>(a)
                           .oldest_local_unreliable_monotonic_timestamp <
                       std::get<1>(b).oldest_local_reliable_monotonic_timestamp;

              } else {
                LOG(FATAL) << "Broken logic, unable to compare timestamps "
                           << std::get<1>(a) << ", " << std::get<1>(b);
              }
            });

        // The last time from the local node on the logger node.
        // This is used to track overlapping boots since this should always
        // increase.
        aos::monotonic_clock::time_point last_boot_time =
            aos::monotonic_clock::min_time;

        // Now take our sorted list and build up constraints so we can solve.
        for (size_t boot_id = 0; boot_id < remote_boot_times.size();
             ++boot_id) {
          const std::tuple<std::string, BootPairTimes, BootPairTimes>
              &boot_time = remote_boot_times[boot_id];
          const std::string &local_boot_uuid = std::get<0>(boot_time);
          VLOG(1) << " Boot " << local_boot_uuid << " is "
                  << std::get<1>(boot_time);

          // Enforce that the last time observed in the headers on the previous
          // boot is less than the first time on the next boot.  This equates to
          // there being no overlap between the two boots.
          if (MinLocalBootTime(std::get<1>(boot_time)) < last_boot_time) {
            for (size_t fatal_boot_id = 0;
                 fatal_boot_id < remote_boot_times.size(); ++fatal_boot_id) {
              const std::tuple<std::string, BootPairTimes, BootPairTimes>
                  &fatal_boot_time = remote_boot_times[fatal_boot_id];
              const std::string &fatal_remote_boot_uuid =
                  std::get<0>(fatal_boot_time);
              LOG(ERROR) << "Boot " << fatal_boot_id << ", "
                         << fatal_remote_boot_uuid << " on " << remote_node_name
                         << " spans ["
                         << MinLocalBootTime(std::get<1>(fatal_boot_time))
                         << ", "
                         << MaxLocalBootTime(std::get<2>(fatal_boot_time))
                         << "] on remote " << remote_node_name;
            }
            LOG(FATAL) << "Broken log, found overlapping boots on "
                       << remote_node_name << " remote node "
                       << remote_node_name << ", "
                       << MinLocalBootTime(std::get<1>(boot_time)) << " < "
                       << last_boot_time;
          }

          last_boot_time = MaxLocalBootTime(std::get<2>(boot_time));

          auto remote_node_boot_constraints_it =
              boot_constraints.find(remote_node_name);
          if (remote_node_boot_constraints_it == boot_constraints.end()) {
            remote_node_boot_constraints_it =
                boot_constraints
                    .insert(std::make_pair(remote_node_name, NodeBootState()))
                    .first;
          }

          // Track that this boot happened.
          remote_node_boot_constraints_it->second.boots.insert(local_boot_uuid);

          if (boot_id > 0) {
            // And now add the constraints.  The vector is in order, so all we
            // need to do is to iterate through it and put pairwise constraints
            // in there.
            std::map<std::string, std::vector<std::pair<std::string, bool>>>
                &per_node_boot_constraints =
                    remote_node_boot_constraints_it->second.constraints;

            const std::tuple<std::string, BootPairTimes, BootPairTimes>
                &prior_boot_time = remote_boot_times[boot_id - 1];
            const std::string &prior_boot_uuid = std::get<0>(prior_boot_time);

            auto first_per_boot_constraints =
                per_node_boot_constraints.find(prior_boot_uuid);
            if (first_per_boot_constraints == per_node_boot_constraints.end()) {
              first_per_boot_constraints =
                  per_node_boot_constraints
                      .insert(std::make_pair(
                          prior_boot_uuid,
                          std::vector<std::pair<std::string, bool>>()))
                      .first;
            }

            auto second_per_boot_constraints =
                per_node_boot_constraints.find(local_boot_uuid);
            if (second_per_boot_constraints ==
                per_node_boot_constraints.end()) {
              second_per_boot_constraints =
                  per_node_boot_constraints
                      .insert(std::make_pair(
                          local_boot_uuid,
                          std::vector<std::pair<std::string, bool>>()))
                      .first;
            }

            VLOG(1) << "Inserting " << first_per_boot_constraints->first
                    << " < " << local_boot_uuid;
            VLOG(1) << "Inserting " << second_per_boot_constraints->first
                    << " > " << prior_boot_uuid;
            first_per_boot_constraints->second.emplace_back(
                std::make_pair(local_boot_uuid, true));
            second_per_boot_constraints->second.emplace_back(
                std::make_pair(prior_boot_uuid, false));
          }
        }
      }
    }
  }

  // Now, sort the reverse direction so we can handle when we only get
  // timestamps and need to make sense of the boots.
  for (const auto &remote_node : reverse_boot_times) {
    const std::string &remote_node_name = remote_node.first;
    for (const auto &remote_node_boot_uuid : remote_node.second) {
      for (const auto &local_node : remote_node_boot_uuid.second) {
        // Now, we need to take all the boots + times and put them in a list to
        // sort and derive constraints from.  This is very similar to the
        // forward direction.

        std::vector<std::pair<std::string, BootPairTimes>> local_boot_times;
        for (const auto &local_boot_uuid : local_node.second) {
          // TODO(austin): If we only have reliable timestamps going the other
          // way, we need to update this logic (and the sorting logic below) to
          // use them.  This should be quite rare, so I feel safe delaying it.
          if (local_boot_uuid.second
                  .oldest_remote_unreliable_monotonic_timestamp ==
              monotonic_clock::max_time) {
            continue;
          }
          local_boot_times.emplace_back(local_boot_uuid);
        }

        std::sort(
            local_boot_times.begin(), local_boot_times.end(),
            [](const std::pair<std::string, BootPairTimes> &a,
               const std::pair<std::string, BootPairTimes> &b) {
              return a.second.oldest_remote_unreliable_monotonic_timestamp <
                     b.second.oldest_remote_unreliable_monotonic_timestamp;
            });

        VLOG(1) << "Reverse sort from " << remote_node_name << " boot "
                << remote_node_boot_uuid.first << " to " << local_node.first;
        for (size_t boot_id = 0; boot_id < local_boot_times.size(); ++boot_id) {
          VLOG(1) << "Sorted local times: " << local_boot_times[boot_id].first
                  << " time " << local_boot_times[boot_id].second;
          const std::pair<std::string, BootPairTimes> &boot_time =
              local_boot_times[boot_id];
          const std::string &local_boot_uuid = boot_time.first;

          auto local_node_boot_constraints_it =
              boot_constraints.find(local_node.first);
          if (local_node_boot_constraints_it == boot_constraints.end()) {
            local_node_boot_constraints_it =
                boot_constraints
                    .insert(std::make_pair(local_node.first, NodeBootState()))
                    .first;
          }

          // Track that this boot happened.
          local_node_boot_constraints_it->second.boots.insert(local_boot_uuid);
          if (boot_id > 0) {
            const std::pair<std::string, BootPairTimes> &prior_boot_time =
                local_boot_times[boot_id - 1];
            const std::string &prior_boot_uuid = prior_boot_time.first;

            std::map<std::string, std::vector<std::pair<std::string, bool>>>
                &per_node_boot_constraints =
                    local_node_boot_constraints_it->second.constraints;

            auto first_per_boot_constraints =
                per_node_boot_constraints.find(prior_boot_uuid);
            if (first_per_boot_constraints == per_node_boot_constraints.end()) {
              first_per_boot_constraints =
                  per_node_boot_constraints
                      .insert(std::make_pair(
                          prior_boot_uuid,
                          std::vector<std::pair<std::string, bool>>()))
                      .first;
            }

            auto second_per_boot_constraints =
                per_node_boot_constraints.find(local_boot_uuid);
            if (second_per_boot_constraints ==
                per_node_boot_constraints.end()) {
              second_per_boot_constraints =
                  per_node_boot_constraints
                      .insert(std::make_pair(
                          local_boot_uuid,
                          std::vector<std::pair<std::string, bool>>()))
                      .first;
            }

            VLOG(1) << "Inserting " << first_per_boot_constraints->first
                    << " < " << local_boot_uuid;
            VLOG(1) << "Inserting " << second_per_boot_constraints->first
                    << " > " << prior_boot_uuid;
            first_per_boot_constraints->second.emplace_back(
                std::make_pair(local_boot_uuid, true));
            second_per_boot_constraints->second.emplace_back(
                std::make_pair(prior_boot_uuid, false));
          }
        }
      }
    }
  }

  return boot_constraints;
}

std::map<std::string, NodeBootState> PartsSorter::ComputeOldBootConstraints() {
  std::map<std::string, NodeBootState> boot_constraints;
  for (std::pair<const std::string, UnsortedLogPartsMap> &logs : parts_list) {
    {
      // We know nothing about the order of the logger node's boot, but we know
      // it happened.  If there is only 1 single boot, the constraint code will
      // happily mark it as boot 0.  Otherwise we'll get the appropriate boot
      // count if it can be computed or an error.
      //
      // Add it to the boots list to kick this off.
      auto it = boot_constraints.find(logs.second.logger_node);
      if (it == boot_constraints.end()) {
        it = boot_constraints
                 .insert(
                     std::make_pair(logs.second.logger_node, NodeBootState()))
                 .first;
      }

      it->second.boots.insert(logs.second.logger_boot_uuid);
    }

    std::map<std::string,
             std::vector<std::pair<std::string, std::pair<int, int>>>>
        node_boot_parts_index_ranges;

    for (std::pair<const std::pair<std::string, std::string>, UnsortedLogParts>
             &parts : logs.second.unsorted_parts) {
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
            << ": Overlapping parts_index, please investigate "
            << boot_parts_index_ranges.first << " "
            << boot_parts_index_ranges.second[i - 1].first << " "
            << boot_parts_index_ranges.second[i].first;
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
  return boot_constraints;
}

MapBoots PartsSorter::ComputeBootCounts() {
  const std::map<std::string, NodeBootState> boot_constraints =
      boot_times.empty() ? ComputeOldBootConstraints()
                         : ComputeNewBootConstraints();

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
      for (const std::pair<const std::string,
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
  MapBoots boots;
  for (const std::pair<const std::string, NodeBootState> &node_state :
       boot_constraints) {
    CHECK_GT(node_state.second.boots.size(), 0u)
        << ": Need a boot from each node.";
    if (node_state.second.boots.size() == 1u) {
      boots.boot_count_map.insert(
          std::make_pair(*node_state.second.boots.begin(), 0));
      boots.boots.insert(std::make_pair(
          node_state.first,
          std::vector<std::string>{*node_state.second.boots.begin()}));
      continue;
    }

    // Now, walk the dag to find the earliest boot.  Pick a node and start
    // traversing.
    std::string current_boot = *node_state.second.boots.begin();
    size_t update_count = 0;

    while (true) {
      auto it = node_state.second.constraints.find(current_boot);
      if (it == node_state.second.constraints.end()) {
        LOG(WARNING) << "Unconnected boot " << current_boot
                     << " in set > 1 for node " << node_state.first;
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
        LOG(WARNING) << "Unconnected boot " << current_boot
                     << " in set > 1 for node " << node_state.first;
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
        << ": Graph failed to reach all the boots on node " << node_state.first;

    VLOG(1) << "Node " << node_state.first;
    size_t boot_count = 0;
    boots.boots.insert(std::make_pair(node_state.first, sorted_boots));
    for (const std::string &boot : sorted_boots) {
      VLOG(1) << "  Boot " << boot;
      boots.boot_count_map.insert(std::make_pair(std::move(boot), boot_count));
      ++boot_count;
    }
  }

  return boots;
}

std::vector<LogFile> PartsSorter::FormatNewParts() {
  // Rewrite MapBoots to Boots since we have enough information here.
  std::shared_ptr<Boots> boot_counts = std::make_shared<Boots>();
  MapBoots map_boot_counts = ComputeBootCounts();
  boot_counts->boot_count_map = std::move(map_boot_counts.boot_count_map);

  // Now, sort them and produce the final vector form.
  std::vector<LogFile> result;
  result.reserve(parts_list.size());
  for (std::pair<const std::string, UnsortedLogPartsMap> &logs : parts_list) {
    LogFile new_file;
    new_file.log_event_uuid = logs.first;
    new_file.logger_node = logs.second.logger_node;
    new_file.logger_boot_uuid = logs.second.logger_boot_uuid;
    new_file.log_start_uuid = logs.second.log_start_uuid;
    new_file.logger_instance_uuid = logs.second.logger_instance_uuid;
    {
      auto boot_count_it =
          boot_counts->boot_count_map.find(new_file.logger_boot_uuid);
      CHECK(boot_count_it != boot_counts->boot_count_map.end());
      new_file.logger_boot_count = boot_count_it->second;
    }
    new_file.monotonic_start_time = logs.second.monotonic_start_time;
    new_file.realtime_start_time = logs.second.realtime_start_time;
    new_file.name = logs.second.name;
    new_file.logger_sha1 = logs.second.logger_sha1;
    new_file.logger_version = logs.second.logger_version;
    new_file.corrupted = corrupted;
    new_file.boots = boot_counts;
    bool seen_part = false;
    std::string config_sha256;
    for (std::pair<const std::pair<std::string, std::string>, UnsortedLogParts>
             &parts : logs.second.unsorted_parts) {
      LogParts new_parts;
      new_parts.monotonic_start_time = parts.second.monotonic_start_time;
      new_parts.realtime_start_time = parts.second.realtime_start_time;
      new_parts.logger_monotonic_start_time =
          parts.second.logger_monotonic_start_time;
      new_parts.logger_realtime_start_time =
          parts.second.logger_realtime_start_time;
      new_parts.log_event_uuid = logs.first;
      new_parts.source_boot_uuid = parts.second.source_boot_uuid;
      new_parts.parts_uuid = parts.first.first;
      new_parts.node = std::move(parts.second.node);
      new_parts.boots = boot_counts;

      {
        auto boot_count_it =
            boot_counts->boot_count_map.find(new_parts.source_boot_uuid);
        CHECK(boot_count_it != boot_counts->boot_count_map.end());
        new_parts.boot_count = boot_count_it->second;
      }
      new_parts.logger_boot_count = new_file.logger_boot_count;

      std::sort(parts.second.parts.begin(), parts.second.parts.end(),
                [](const std::pair<std::string, int> &a,
                   const std::pair<std::string, int> &b) {
                  return a.second < b.second;
                });
      new_parts.parts.reserve(parts.second.parts.size());
      {
        int last_parts_index = -1;
        std::string_view last_part_name;
        for (std::pair<std::string, int> &p : parts.second.parts) {
          CHECK_LT(last_parts_index, p.second)
              << ": Broken log, Found duplicate parts in '" << last_part_name
              << "' and '" << p.first << "'";
          if (last_parts_index != -1) {
            if (p.second != last_parts_index + 1) {
              LOG(FATAL) << "Broken log, missing part files between \""
                         << last_part_name << "\" and \"" << p.first
                         << "\", found "
                         << ConcatenateParts(parts.second.parts);
            }
          }
          last_parts_index = p.second;
          last_part_name = p.first;
        }

        // Now that we are happy that it works, move them all.
        for (std::pair<std::string, int> &p : parts.second.parts) {
          new_parts.parts.emplace_back(std::move(p.first));
        }
      }

      CHECK(!parts.second.config_sha256.empty());
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
      seen_part = true;

      new_file.parts.emplace_back(std::move(new_parts));
    }
    result.emplace_back(std::move(new_file));
  }

  {
    CHECK_EQ(config_sha256_lookup.size(), 1u)
        << ": We only support log files with 1 config in them.";
    std::shared_ptr<const aos::Configuration> config =
        config_sha256_lookup.begin()->second;

    boot_counts->boots.resize(configuration::NodesCount(config.get()));
    for (std::pair<const std::string, std::vector<std::string>> &boots :
         map_boot_counts.boots) {
      size_t node_index = 0;
      if (configuration::MultiNode(config.get())) {
        node_index = configuration::GetNodeIndex(config.get(), boots.first);
      }

      boot_counts->boots[node_index] = std::move(boots.second);
    }
  }

  return result;
}

std::vector<LogFile> PartsSorter::SortParts() {
  if (old_parts.empty() && parts_list.empty()) {
    if (!corrupted.empty()) {
      LogFile log_file;
      log_file.corrupted = std::move(corrupted);
      return std::vector<LogFile>{log_file};
    }
    return std::vector<LogFile>{};
  }
  CHECK_NE(old_parts.empty(), parts_list.empty())
      << ": Can't have a mix of old and new parts.";
  if (!old_parts.empty()) {
    return FormatOldParts();
  } else {
    return FormatNewParts();
  }
}

std::vector<LogFile> SortParts(const std::vector<std::string> &parts) {
  LogReadersPool readers;
  PartsSorter sorter;
  sorter.PopulateFromFiles(&readers, parts);
  return sorter.SortParts();
}

std::vector<LogFile> SortParts(const LogSource &log_source) {
  LogReadersPool readers(&log_source);
  PartsSorter sorter;
  sorter.PopulateFromFiles(&readers, log_source.ListFiles());
  return sorter.SortParts();
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
  if (!file.logger_instance_uuid.empty()) {
    stream << " \"logger_instance_uuid\": \"" << file.logger_instance_uuid
           << "\",\n";
  }
  if (!file.log_start_uuid.empty()) {
    stream << " \"log_start_uuid\": \"" << file.log_start_uuid << "\",\n";
  }
  if (!file.name.empty()) {
    stream << " \"name\": \"" << file.name << "\",\n";
  }
  if (!file.logger_sha1.empty()) {
    stream << " \"logger_sha1\": \"" << file.logger_sha1 << "\",\n";
  }
  if (!file.logger_version.empty()) {
    stream << " \"logger_version\": \"" << file.logger_version << "\",\n";
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

// Validates that collection of log files or log parts shares the same configs.
template <typename TCollection>
bool CheckMatchingConfigs(const TCollection &items) {
  const Configuration *config = nullptr;
  for (const auto &item : items) {
    VLOG(1) << item;
    if (config == nullptr) {
      config = GetConfig(item);
    } else {
      if (config != GetConfig(item)) {
        LOG(ERROR) << ": Config mismatched: " << config << " vs. "
                   << GetConfig(item);
        return false;
      }
    }
  }
  if (config == nullptr) {
    LOG(ERROR) << ": No configs are found";
    return false;
  }
  return true;
}

// Provides unified access to config field stored in LogFile. It is used in
// CheckMatchingConfigs.
inline const Configuration *GetConfig(const LogFile &log_file) {
  return log_file.config.get();
}

// Output of LogPartsAccess for debug purposes.
std::ostream &operator<<(std::ostream &stream,
                         const LogPartsAccess &log_parts_access) {
  stream << log_parts_access.parts();
  return stream;
}

SelectedLogParts::SelectedLogParts(std::string_view node_name,
                                   size_t boot_index,
                                   std::vector<LogPartsAccess> log_parts)
    : node_name_(node_name),
      boot_index_(boot_index),
      log_parts_(std::move(log_parts)) {
  CHECK_GT(log_parts_.size(), 0u) << ": Nothing was selected for node "
                                  << node_name_ << " boot " << boot_index_;
  CHECK(CheckMatchingConfigs(log_parts_));
  config_ = log_parts_.front().config();

  // Enforce that we are sorting things only from a single node from a single
  // boot.
  const std::string_view part0_source_boot_uuid =
      log_parts_.front().source_boot_uuid();
  for (const auto &part : log_parts_) {
    CHECK_EQ(node_name_, part.node_name()) << ": Can't merge different nodes.";
    CHECK_EQ(part0_source_boot_uuid, part.source_boot_uuid())
        << ": Can't merge different boots.";
    CHECK_EQ(boot_index_, part.boot_count());
  }
}

LogFilesContainer::LogFilesContainer(
    std::optional<const LogSource *> log_source, std::vector<LogFile> log_files)
    : log_source_(log_source), log_files_(std::move(log_files)) {
  CHECK_GT(log_files_.size(), 0u);
  CHECK(CheckMatchingConfigs(log_files_));
  config_ = log_files_.front().config.get();
  boots_ = log_files_.front().boots;

  std::unordered_set<std::string> logger_nodes;

  // Scan and collect all related nodes and number of reboots per node.
  for (const LogFile &log_file : log_files_) {
    for (const LogParts &part : log_file.parts) {
      auto node_item = nodes_boots_.find(part.node);
      if (node_item != nodes_boots_.end()) {
        node_item->second = std::max(node_item->second, part.boot_count + 1);
      } else {
        nodes_boots_[part.node] = part.boot_count + 1;
      }
    }
    logger_nodes.insert(log_file.logger_node);
  }
  while (!logger_nodes.empty()) {
    logger_nodes_.emplace_back(
        logger_nodes.extract(logger_nodes.begin()).value());
  }
}

size_t LogFilesContainer::BootsForNode(std::string_view node_name) const {
  const auto &node_item = nodes_boots_.find(std::string(node_name));
  CHECK(node_item != nodes_boots_.end())
      << ": Missing parts associated with node " << node_name;
  CHECK_GT(node_item->second, 0u) << ": No boots for node " << node_name;
  return node_item->second;
}

SelectedLogParts LogFilesContainer::SelectParts(std::string_view node_name,
                                                size_t boot_index) const {
  std::vector<LogPartsAccess> result;
  for (const LogFile &log_file : log_files_) {
    for (const LogParts &part : log_file.parts) {
      if (part.node == node_name && part.boot_count == boot_index) {
        result.emplace_back(log_source_, part);
      }
    }
  }
  return SelectedLogParts(node_name, boot_index, result);
}

}  // namespace logger
}  // namespace aos
