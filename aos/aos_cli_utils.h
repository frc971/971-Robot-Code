#ifndef AOS_AOS_CLI_UTILS_H_
#define AOS_AOS_CLI_UTILS_H_

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "gflags/gflags.h"

namespace aos {

struct PrintOptions {
  // Format the JSON with the pretty option.
  bool pretty;
  // Max vector size to skip expanding.
  size_t max_vector_size;
  // Put everything on a separate line instead of keeping small messages
  // together.
  bool pretty_max;
  // Print the timestamps.
  bool print_timestamps;
  // Make everything JSON compliant.
  bool json;
  // Print the distributed clock.
  bool distributed_clock;
  // Print numbers out in hex.
  bool use_hex;
};

// Print the flatbuffer out to stdout, both to remove the unnecessary cruft from
// glog and to allow the user to readily redirect just the logged output
// independent of any debugging information on stderr.
void PrintMessage(const std::string_view node_name,
                  aos::NodeEventLoopFactory *node_factory,
                  const aos::Channel *channel, const aos::Context &context,
                  aos::FastStringBuilder *builder, PrintOptions options);

void PrintMessage(const aos::Channel *channel, const aos::Context &context,
                  aos::FastStringBuilder *builder, PrintOptions options);

// RAII class to manage printing out sequences of messages, and to print them
// all in a JSON array properly.
class Printer {
 public:
  Printer(PrintOptions options, bool flush);
  ~Printer();

  // Number of messages that have been printed.
  uint64_t message_count() const { return message_count_; }

  // Prints a message.
  void PrintMessage(const std::string_view node_name,
                    aos::NodeEventLoopFactory *node_factory,
                    const aos::Channel *channel, const aos::Context &context);
  void PrintMessage(const aos::Channel *channel, const aos::Context &context);

 private:
  // Builder to make printing fast
  aos::FastStringBuilder str_builder_;

  // Number of messages
  uint64_t message_count_ = 0;

  // Options for printing.
  const PrintOptions options_;

  // If true, use std::endl to flush stdout, otherwise write a "\n"
  const bool flush_;
};

// The information needed by the main function of a CLI tool.
struct CliUtilInfo {
  // If this returns true, main should return immediately with 0.
  // If this returns false, the other fields will be filled out appropriately.
  // event_loop will be filled out before channel_filter is called.
  bool Initialize(int *argc, char ***argv,
                  std::function<bool(const aos::Channel *)> channel_filter,
                  std::string_view channel_filter_description,
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
