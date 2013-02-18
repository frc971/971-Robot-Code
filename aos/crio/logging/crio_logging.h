#ifndef AOS_CRIO_CRIO_LOGGING_LOGGING_H_
#define AOS_CRIO_CRIO_LOGGING_LOGGING_H_

namespace aos {
namespace logging {
namespace crio {

// Calls AddImplementation to register the usual crio logging implementation
// which sends the messages to the atom. This function will start the task that
// does the actual sending. It relies on a process being running on the atom to
// receive them.
void Register();

}  // namespace crio
}  // namespace logging
}  // namespace aos

#endif  // AOS_CRIO_CRIO_LOGGING_LOGGING_H_
