#ifndef AOS_LINUX_CODE_INIT_H_
#define AOS_LINUX_CODE_INIT_H_

#include <string>

namespace aos {

// In order to use shared memory, one of the Init* functions must be called in
// exactly 1 thread per process. It is OK to keep going without calling one of
// them again after fork(2)ing.

// Does the non-realtime parts of the initialization process.
void InitNRT();
// Initializes everything, including the realtime stuff.
// relative_priority adjusts the priority of this process relative to all of the
// other ones (positive for higher priority).
void Init(int relative_priority = 0);
// Same as InitNRT, except will remove an existing shared memory file and create
// a new one.
void InitCreate();
// Cleans up (probably not going to get called very often because few things can
// exit gracefully).
void Cleanup();

// Sets up this process to write core dump files.
// This is called by Init*, but it's here for other files that want this
// behavior without calling Init*.
void WriteCoreDumps();

// Sets the current thread's realtime priority.
void SetCurrentThreadRealtimePriority(int priority);

// Sets the current thread back down to non-realtime priority.
void UnsetCurrentThreadRealtimePriority();

// Pins the current thread to CPU #number.
void PinCurrentThreadToCPU(int number);

// Sets the name of the current thread.
// This will displayed by `top -H`, dump_rtprio, and show up in logs.
// name can have a maximum of 16 characters.
void SetCurrentThreadName(const ::std::string &name);

void LockAllMemory();

}  // namespace aos

#endif  // AOS_LINUX_CODE_INIT_H_
