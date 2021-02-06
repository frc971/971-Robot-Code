#ifndef AOS_EVENTS_LOGGING_LOGFILE_SORTING_H_
#define AOS_EVENTS_LOGGING_LOGFILE_SORTING_H_

#include <iostream>
#include <string>
#include <vector>

#include "aos/configuration.h"
#include "aos/events/logging/uuid.h"
#include "aos/time/time.h"

namespace aos {
namespace logger {

// Datastructure to hold ordered parts.
struct LogParts {
  // Monotonic and realtime start times for this set of log files.  For log
  // files which started out unknown and then became known, this is the known
  // start time.
  aos::monotonic_clock::time_point monotonic_start_time;
  aos::realtime_clock::time_point realtime_start_time;

  // Time on the logger node (if applicable) that this log file started.
  aos::monotonic_clock::time_point logger_monotonic_start_time =
      aos::monotonic_clock::min_time;
  aos::realtime_clock::time_point logger_realtime_start_time =
      aos::realtime_clock::min_time;

  // UUIDs if available.
  std::string log_event_uuid;
  std::string parts_uuid;

  // The node this represents, or empty if we are in a single node world.
  std::string node;

  // Boot UUID of the node which generated this data, if available.  For local
  // data and timestamps, this is the same as the logger_boot_uuid.  For remote
  // data, this is the boot_uuid of the remote node.
  std::string source_boot_uuid;

  // Pre-sorted list of parts.
  std::vector<std::string> parts;

  // Configuration for all the log parts.  This will be a single object for all
  // log files with the same config.
  std::string config_sha256;
  std::shared_ptr<const aos::Configuration> config;
};

// Datastructure to hold parts from the same run of the logger which have no
// ordering constraints relative to each other.
struct LogFile {
  // The UUID tying them all together (if available)
  std::string log_event_uuid;

  // The node the logger was running on (if available)
  std::string logger_node;
  // Boot UUID of the node running the logger.
  std::string logger_boot_uuid;

  // The start time on the logger node.
  aos::monotonic_clock::time_point monotonic_start_time;
  aos::realtime_clock::time_point realtime_start_time;

  // The name field in the log file headers.
  std::string name;

  // All the parts, unsorted.
  std::vector<LogParts> parts;

  // A list of parts which were corrupted and are unknown where they should go.
  std::vector<std::string> corrupted;

  // Configuration for all the log parts and files.  This will be a single
  // object for log files with the same config.
  std::string config_sha256;
  std::shared_ptr<const aos::Configuration> config;
};

std::ostream &operator<<(std::ostream &stream, const LogFile &file);
std::ostream &operator<<(std::ostream &stream, const LogParts &parts);

// Takes a bunch of parts and sorts them based on part_uuid and part_index.
std::vector<LogFile> SortParts(const std::vector<std::string> &parts);

// Finds all the nodes which have parts logged from their point of view.
std::vector<std::string> FindNodes(const std::vector<LogFile> &parts);
// Finds all the parts which are from the point of view of a single node.
std::vector<LogParts> FilterPartsForNode(const std::vector<LogFile> &parts,
                                         std::string_view node);

// Recursively searches the file/folder for .bfbs and .bfbs.xz files and adds
// them to the vector.
void FindLogs(std::vector<std::string> *files, std::string filename);

// Recursively searches for logfiles in argv[1] and onward.
std::vector<std::string> FindLogs(int argc, char **argv);

// Returns the sha256 of a span.
std::string Sha256(const absl::Span<const uint8_t> str);

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_LOGFILE_SORTING_H_
