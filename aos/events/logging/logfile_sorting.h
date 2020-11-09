#ifndef AOS_EVENTS_LOGGING_LOGFILE_SORTING_H_
#define AOS_EVENTS_LOGGING_LOGFILE_SORTING_H_

#include <iostream>
#include <vector>
#include <string>

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

  // UUIDs if available.
  std::string log_event_uuid;
  std::string parts_uuid;

  // The node this represents, or empty if we are in a single node world.
  std::string node;

  // Pre-sorted list of parts.
  std::vector<std::string> parts;
};

// Datastructure to hold parts from the same run of the logger which have no
// ordering constraints relative to each other.
struct LogFile {
  // The UUID tying them all together (if available)
  std::string log_event_uuid;

  // The node the logger was running on (if available)
  std::string logger_node;

  // The start time on the logger node.
  aos::monotonic_clock::time_point monotonic_start_time;
  aos::realtime_clock::time_point realtime_start_time;

  // All the parts, unsorted.
  std::vector<LogParts> parts;

  // A list of parts which were corrupted and are unknown where they should go.
  std::vector<std::string> corrupted;
};

std::ostream &operator<<(std::ostream &stream, const LogFile &file);
std::ostream &operator<<(std::ostream &stream, const LogParts &parts);

// Takes a bunch of parts and sorts them based on part_uuid and part_index.
std::vector<LogFile> SortParts(const std::vector<std::string> &parts);

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_LOGFILE_SORTING_H_
