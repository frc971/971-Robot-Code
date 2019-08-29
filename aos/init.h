#ifndef AOS_INIT_H_
#define AOS_INIT_H_

#include <string>

namespace aos {

// Initializes glog and gflags.
void InitGoogle(int *argc, char ***argv);

// In order to use shared memory, one of the Init* functions must be called in
// exactly 1 thread per process. It is OK to keep going without calling one of
// them again after fork(2)ing.

// Does the non-realtime parts of the initialization process.
// If for_realtime is true, this sets up to call GoRT later.
void InitNRT(bool for_realtime = false);
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

// Performs the realtime parts of initialization after InitNRT(true) has been called.
void GoRT(int relative_priority = 0);

// Pins the current thread to CPU #number.
void PinCurrentThreadToCPU(int number);

}  // namespace aos

#endif  // AOS_INIT_H_
