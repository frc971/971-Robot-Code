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

std::vector<LogFile> SortParts(const std::vector<std::string> &parts) {
  std::vector<std::string> corrupted;

  std::map<std::string, std::shared_ptr<const Configuration>>
      config_sha256_lookup;

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

  // Map holding the log_event_uuid -> second map.  The second map holds the
  // parts_uuid -> list of parts for sorting.
  std::map<std::string, UnsortedLogPartsMap> parts_list;

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

  // A list of all the old parts which we don't know how to sort using uuids.
  // There are enough of these in the wild that this is worth supporting.
  std::vector<UnsortedOldParts> old_parts;

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
      CHECK(log_header->message().has_configuration());
    } else {
      CHECK(!log_header->message().has_configuration());
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

  if (old_parts.empty() && parts_list.empty()) {
    if (parts.empty()) {
      return std::vector<LogFile>{};
    } else {
      LogFile log_file;
      log_file.corrupted = std::move(corrupted);
      return std::vector<LogFile>{log_file};
    }
  }
  CHECK_NE(old_parts.empty(), parts_list.empty())
      << ": Can't have a mix of old and new parts.";

  // Now reformat old_parts to be in the right datastructure to report.
  std::map<std::string, std::shared_ptr<const Configuration>>
      copied_config_sha256;
  if (!old_parts.empty()) {
    std::vector<LogFile> result;
    for (UnsortedOldParts &p : old_parts) {
      // Sort by the oldest message in each file.
      std::sort(
          p.unsorted_parts.begin(), p.unsorted_parts.end(),
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
      auto header =
          std::make_shared<SizePrefixedFlatbufferVector<LogFileHeader>>(
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
  stream << " \"config\": " << file.config.get();
  if (!file.config_sha256.empty()) {
    stream << ",\n \"config_sha256\": \"" << file.config_sha256 << "\"";
  }
  stream << ",\n \"monotonic_start_time\": " << file.monotonic_start_time
         << ",\n \"realtime_start_time\": " << file.realtime_start_time
         << ",\n";
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
  }
  stream << "  \"config\": " << parts.config.get();
  if (!parts.config_sha256.empty()) {
    stream << ",\n  \"config_sha256\": \"" << parts.config_sha256 << "\"";
  }
  stream << ",\n  \"monotonic_start_time\": " << parts.monotonic_start_time
         << ",\n  \"realtime_start_time\": " << parts.realtime_start_time
         << ",\n  \"parts\": [";

  for (size_t i = 0; i < parts.parts.size(); ++i) {
    if (i != 0u) {
      stream << ", ";
    }
    stream << parts.parts[i];
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
