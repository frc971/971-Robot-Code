#ifndef AOS_INIT_H_
#define AOS_INIT_H_

namespace aos {

// Initializes AOS.
void InitGoogle(int *argc, char ***argv);

// Returns true if we have been initialized.  This is mostly here so
// ShmEventLoop can confirm the world was initialized before running.
bool IsInitialized();

}  // namespace aos

#endif  // AOS_INIT_H_
