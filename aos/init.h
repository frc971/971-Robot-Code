#ifndef AOS_INIT_H_
#define AOS_INIT_H_

namespace aos {

// Initializes AOS.
void InitGoogle(int *argc, char ***argv);

// Returns true if we have been initialized.  This is mostly here so
// ShmEventLoop can confirm the world was initialized before running.
bool IsInitialized();

// A special initialization function that initializes the C++ parts in a way
// compatible with Rust. This requires careful coordination with `:init_rs`, do
// not use it from anywhere else.
void InitFromRust(const char *argv0);

}  // namespace aos

#endif  // AOS_INIT_H_
