#ifndef AOS_LINUX_CODE_INIT_H_
#define AOS_LINUX_CODE_INIT_H_

namespace aos {

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

}  // namespace aos

#endif  // AOS_LINUX_CODE_INIT_H_
