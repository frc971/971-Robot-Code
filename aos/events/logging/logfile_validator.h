#ifndef AOS_EVENTS_LOGGING_LOGFILE_VALIDATOR_H_
#define AOS_EVENTS_LOGGING_LOGFILE_VALIDATOR_H_
#include "aos/events/logging/logfile_sorting.h"
namespace aos::logger {
// Attempts to validate that a log is readable without actually running the full
// LogReader. This aims to allow the user to preempt fatal crashes that can
// occur when trying to replay a log.
// Returns true if successful.
bool MultiNodeLogIsReadable(const LogFilesContainer &log_files,
                            bool skip_order_validation = false);

// Returns true if the requested log is either a single-node log or if the
// MultiNodeLogIsReadable() returns true.
bool LogIsReadableIfMultiNode(const LogFilesContainer &log_files);
}  // namespace aos::logger
#endif  // AOS_EVENTS_LOGGING_LOGFILE_VALIDATOR_H_
