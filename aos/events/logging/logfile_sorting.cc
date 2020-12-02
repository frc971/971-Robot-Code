#include "aos/events/logging/logfile_sorting.h"

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "aos/events/logging/logfile_utils.h"
#include "aos/flatbuffers.h"
#include "aos/time/time.h"

namespace aos {
namespace logger {
namespace chrono = std::chrono;

std::vector<LogFile> SortParts(const std::vector<std::string> &parts) {
  std::vector<std::string> corrupted;

  // Start by grouping all parts by UUID, and extracting the part index.
  // Datastructure to hold all the info extracted from a set of parts which go
  // together so we can sort them afterwords.
  struct UnsortedLogParts {
    // Start times.
    aos::monotonic_clock::time_point monotonic_start_time;
    aos::realtime_clock::time_point realtime_start_time;

    // Node to save.
    std::string node;

    // Pairs of the filename and the part index for sorting.
    std::vector<std::pair<std::string, int>> parts;
  };

  // Struct to hold both the node, and the parts associated with it.
  struct UnsortedLogPartsMap {
    std::string logger_node;
    aos::monotonic_clock::time_point monotonic_start_time =
        aos::monotonic_clock::min_time;
    aos::realtime_clock::time_point realtime_start_time =
        aos::realtime_clock::min_time;

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

    const std::string_view node =
        log_header->message().has_node()
            ? log_header->message().node()->name()->string_view()
            : "";

    const std::string_view logger_node =
        log_header->message().has_logger_node()
            ? log_header->message().logger_node()->name()->string_view()
            : "";

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
      } else {
        result->unsorted_parts.emplace_back(
            std::make_pair(first_message_time, part));
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
    } else {
      CHECK_EQ(log_it->second.logger_node, logger_node);
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
      it->second.node = std::string(node);
    }

    // First part might be min_time.  If it is, try to put a better time on it.
    if (it->second.monotonic_start_time == monotonic_clock::min_time) {
      it->second.monotonic_start_time = monotonic_start_time;
    } else if (monotonic_start_time != monotonic_clock::min_time) {
      CHECK_EQ(it->second.monotonic_start_time, monotonic_start_time);
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
      for (std::pair<monotonic_clock::time_point, std::string> &f :
           p.unsorted_parts) {
        p.parts.parts.emplace_back(std::move(f.second));
      }
      log_file.parts.emplace_back(std::move(p.parts));
      log_file.monotonic_start_time = log_file.parts[0].monotonic_start_time;
      log_file.realtime_start_time = log_file.parts[0].realtime_start_time;
      log_file.corrupted = corrupted;
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
    new_file.monotonic_start_time = logs.second.monotonic_start_time;
    new_file.realtime_start_time = logs.second.realtime_start_time;
    new_file.corrupted = corrupted;
    for (std::pair<const std::string, UnsortedLogParts> &parts :
         logs.second.unsorted_parts) {
      LogParts new_parts;
      new_parts.monotonic_start_time = parts.second.monotonic_start_time;
      new_parts.realtime_start_time = parts.second.realtime_start_time;
      new_parts.log_event_uuid = logs.first;
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
      new_file.parts.emplace_back(std::move(new_parts));
    }
    result.emplace_back(std::move(new_file));
  }
  return result;
}

std::vector<std::string> FindNodes(const std::vector<LogFile> &parts) {
  std::set<std::string> nodes;
  for (const LogFile &log_file : parts) {
    for (const LogParts& part : log_file.parts) {
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
    for (const LogParts& part : log_file.parts) {
      if (part.node == node) {
        result.emplace_back(part);
      }
    }
  }
  return result;
}

std::ostream &operator<<(std::ostream &stream, const LogFile &file) {
  stream << "{";
  if (!file.log_event_uuid.empty()) {
    stream << "\"log_event_uuid\": \"" << file.log_event_uuid << "\", ";
  }
  if (!file.logger_node.empty()) {
    stream << "\"logger_node\": \"" << file.logger_node << "\", ";
  }
  stream << "\"monotonic_start_time\": " << file.monotonic_start_time
         << ", \"realtime_start_time\": " << file.realtime_start_time << ", [";
  stream << "\"parts\": [";
  for (size_t i = 0; i < file.parts.size(); ++i) {
    if (i != 0u) {
      stream << ", ";
    }
    stream << file.parts[i];
  }
  stream << "]}";
  return stream;
}
std::ostream &operator<<(std::ostream &stream, const LogParts &parts) {
  stream << "{";
  if (!parts.log_event_uuid.empty()) {
    stream << "\"log_event_uuid\": \"" << parts.log_event_uuid << "\", ";
  }
  if (!parts.parts_uuid.empty()) {
    stream << "\"parts_uuid\": \"" << parts.parts_uuid << "\", ";
  }
  if (!parts.node.empty()) {
    stream << "\"node\": \"" << parts.node << "\", ";
  }
  stream << "\"monotonic_start_time\": " << parts.monotonic_start_time
         << ", \"realtime_start_time\": " << parts.realtime_start_time << ", [";

  for (size_t i = 0; i < parts.parts.size(); ++i) {
    if (i != 0u) {
      stream << ", ";
    }
    stream << parts.parts[i];
  }

  stream << "]}";
  return stream;
}

}  // namespace logger
}  // namespace aos
