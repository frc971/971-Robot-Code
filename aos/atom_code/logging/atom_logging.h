#ifndef AOS_ATOM_CODE_LOGGING_LOGGING_H_
#define AOS_ATOM_CODE_LOGGING_LOGGING_H_

#include "aos/common/logging/logging_impl.h"

namespace aos {
namespace logging {
namespace atom {

// Calls AddImplementation to register the usual atom logging implementation
// which sends the messages through a queue. This implementation relies on
// another process(es) to read the log messages that it puts into the queue.
// It gets called by aos::Init*.
void Register();

// Fairly simple wrappers around the raw queue calls.

// This one never returns NULL if flags contains BLOCK.
const LogMessage *ReadNext(int flags);
const LogMessage *ReadNext(int flags, int *index);
const LogMessage *ReadNext();
LogMessage *Get();
void Free(const LogMessage *msg);
void Write(LogMessage *msg);

}  // namespace atom
}  // namespace logging
}  // namespace aos

#endif
