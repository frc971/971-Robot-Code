#ifndef AOS_AOS_CLI_UTILS_H_
#define AOS_AOS_CLI_UTILS_H_

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "gflags/gflags.h"

namespace aos {

// The information needed by the main function of a CLI tool.
struct CliUtilInfo {
  // If this returns true, main should return immediately with 0.
  // If this returns false, the other fields will be filled out appropriately.
  // event_loop will be filled out before channel_filter is called.
  bool Initialize(int *argc, char ***argv,
                  std::function<bool(const aos::Channel *)> channel_filter,
                  bool expect_args);

  std::optional<aos::FlatbufferDetachedBuffer<aos::Configuration>> config;
  std::optional<aos::ShmEventLoop> event_loop;
  std::vector<const aos::Channel *> found_channels;

 private:
  // Generate eval command to populate autocomplete responses. Eval escapes
  // spaces so channels are paired with their types. If a complete channel name
  // is found, only autocompletes the type to avoid repeating arguments. Returns
  // no autocomplete suggestions if a channel and type is found with the current
  // arguments.
  void Autocomplete(std::string_view channel_name,
                    std::string_view message_type,
                    std::function<bool(const aos::Channel *)> channel_filter);

  void ShiftArgs(int *argc, char ***argv) {
    for (int i = 1; i + 1 < *argc; ++i) {
      (*argv)[i] = (*argv)[i + 1];
    }
    --*argc;
  }
};

}  // namespace aos

#endif  // AOS_AOS_CLI_UTILS_H_
