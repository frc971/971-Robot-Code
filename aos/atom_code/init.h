#ifndef AOS_ATOM_CODE_INIT_H_
#define AOS_ATOM_CODE_INIT_H_

namespace aos {

// Does the non-realtime parts of the initialization process.
void InitNRT();
// Initializes everything, including the realtime stuff.
void Init();
// Same as InitNRT, except will remove an existing shared memory file and create
// a new one.
void InitCreate();
// Cleans up (probably not going to get called very often because few things can
// exit gracefully).
void Cleanup();

}  // namespace aos

#endif  // AOS_ATOM_CODE_INIT_H_
