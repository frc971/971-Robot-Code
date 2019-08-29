#ifndef AOS_REALTIME_H_
#define AOS_REALTIME_H_

#include <string>

namespace aos {

// Locks everything into memory and sets the limits.  This plus InitNRT are
// everything you need to do before SetCurrentThreadRealtimePriority will make
// your thread RT.  Called as part of ShmEventLoop::Run()
void InitRT();

// Sets the current thread back down to non-realtime priority.
void UnsetCurrentThreadRealtimePriority();

// Sets the name of the current thread.
// This will displayed by `top -H`, dump_rtprio, and show up in logs.
// name can have a maximum of 16 characters.
void SetCurrentThreadName(const ::std::string &name);

// Sets the current thread's realtime priority.
void SetCurrentThreadRealtimePriority(int priority);

// Sets up this process to write core dump files.
// This is called by Init*, but it's here for other files that want this
// behavior without calling Init*.
void WriteCoreDumps();

void LockAllMemory();

}  // namespace aos

#endif  // AOS_REALTIME_H_
