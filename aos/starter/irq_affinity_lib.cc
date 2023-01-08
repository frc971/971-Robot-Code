#include "aos/starter/irq_affinity_lib.h"

#include <fcntl.h>

#include "absl/strings/escaping.h"
#include "absl/strings/match.h"
#include "absl/strings/str_split.h"
#include "aos/scoped/scoped_fd.h"

namespace aos {

// Class to split strings by whitespace with absl::StrSplit.
class ByWhitespace {
 public:
  // Returns the location of the next separator per the StrSplit API.
  absl::string_view Find(absl::string_view text, size_t pos) const {
    size_t count = 0;
    const char *start = text.data() + text.size();
    for (size_t i = pos; i < text.size(); ++i) {
      if (text[i] == ' ') {
        if (count == 0) {
          start = text.data() + i;
        }
        ++count;
      } else {
        if (count > 0) {
          break;
        }
      }
    }
    return std::string_view(start, count);
  }
};

InterruptsStatus::InterruptsStatus() {
  // 8k seems to be big enough to hold most things, so let's start there.
  interrupts_content_.reserve(8192);
}

namespace {

// Counts the number of "CPU" strings in the line without allocating.
size_t CountCores(std::string_view line) {
  size_t cpus_found = 0;
  std::string_view::size_type start_pos = 0;
  while (std::string_view::npos != (start_pos = line.find("CPU", start_pos))) {
    start_pos += 3;
    ++cpus_found;
  }
  return cpus_found;
}

}  // namespace

void InterruptsStatus::Update(std::string_view contents) {
  size_t line_number = 0;
  for (std::string_view line :
       absl::StrSplit(contents, "\n", absl::SkipEmpty())) {
    if (line_number == 0) {
      // This is the CPUs line.  Count our cores.
      const size_t cpus_found = CountCores(line);

      if (cpus_ == 0) {
        // First time through.
        cpus_ = cpus_found;
      } else {
        CHECK_EQ(cpus_found, cpus_) << ": Number of CPUs changed while running";
      }
    } else {
      size_t element_number = 0;
      InterruptState *state = nullptr;
      bool new_element = false;
      for (const std::string_view element :
           absl::StrSplit(absl::StripAsciiWhitespace(line),
                          absl::MaxSplits(ByWhitespace(), cpus_ + 1))) {
        if (element_number == 0) {
          // Parse the interrupt number.  This should either be in the form of:
          //  23:
          // or
          //  Err:
          CHECK_EQ(element[element.size() - 1], ':')
              << ": Missing trailing ':'";

          int interrupt_number;
          std::string_view interrupt_name =
              element.substr(0, element.size() - 1);

          // It's a named interrupt.
          if (!absl::SimpleAtoi(interrupt_name, &interrupt_number)) {
            interrupt_number = -1;
          } else {
            interrupt_name = "";
          }

          // Add a new element if we are too short, or if the interrupt changed.
          if (states_.size() < line_number) {
            new_element = true;
          } else {
            InterruptState *state = &states_[line_number - 1];
            if (state->interrupt_number != interrupt_number ||
                state->interrupt_name != interrupt_name) {
              VLOG(1)
                  << "IRQ changed names...  Blow away the end and try again.";
              // This happens infrequently enough that it isn't worth trying to
              // resize things.  It may never happen while running.  Nuke
              // anything missing and retry.
              states_.resize(line_number - 1);
              new_element = true;
            }
          }

          if (new_element) {
            InterruptState new_state;
            std::vector<unsigned int> irq_count(cpus_, 0);
            states_.push_back(InterruptState{
                .interrupt_number = interrupt_number,
                .interrupt_name = interrupt_number == -1
                                      ? std::string(interrupt_name)
                                      : std::string(),
                .count = std::move(irq_count),
                .chip_name = std::string(),
                .description = std::string(),
                .hwirq = std::string(),
                .actions = {},
            });
          }
          state = &states_[line_number - 1];

        } else if (element_number <= cpus_) {
          // We are now parsing the count body.  Keep updating the elements.
          unsigned int interrupt_count;
          CHECK(absl::SimpleAtoi(element, &interrupt_count))
              << ": Failed to parse count " << interrupt_count;
          state->count[element_number - 1] = interrupt_count;
        } else if (element_number == cpus_ + 1) {
          if (state->interrupt_number == -1) {
            // Named interrupt, the rest of the string is the description.
            if (new_element) {
              state->description = std::string(element);
            } else {
              CHECK_EQ(state->description, element) << ": Description changed";
            }
          } else {
            // Ok, the rest is now some properties of the interrupt.
            size_t trailing_elements_count = 0;
            for (std::string_view trailing_element :
                 absl::StrSplit(absl::StripAsciiWhitespace(element),
                                absl::MaxSplits(ByWhitespace(), 2))) {
              if (trailing_elements_count == 0) {
                // Chip name.
                if (new_element) {
                  state->chip_name = std::string(trailing_element);
                } else {
                  CHECK_EQ(state->chip_name, trailing_element)
                      << ": Chip changed names";
                }
              } else if (trailing_elements_count == 1) {
                // Hardware IRQ
                if (new_element) {
                  state->hwirq = std::string(trailing_element);
                } else {
                  CHECK_EQ(state->hwirq, trailing_element)
                      << ": Hardware IRQ changed names";
                }
              } else {
                // And then either "Level/Edge" and then the actions, or just
                // the actions. Kernel has CONFIG_GENERIC_IRQ_SHOW_LEVEL
                // enabled if the string starts with either..  Strip it until someone finds a use.
                if (absl::StartsWith(trailing_element, "Level")) {
                  trailing_element =
                      absl::StripAsciiWhitespace(trailing_element.substr(5));
                } else if (absl::StartsWith(trailing_element, "Edge")) {
                  trailing_element =
                      absl::StripAsciiWhitespace(trailing_element.substr(4));
                }

                // Split up the actions by ", " and stick them in.
                if (new_element) {
                  state->actions = std::vector<std::string>(
                      absl::StrSplit(trailing_element, ", "));
                } else {
                  size_t action_index = 0;
                  bool matches = true;
                  // Start by comparing.  If we don't match, then set.  This
                  // avoids an allocation if we can get away with it.
                  for (std::string_view action :
                       absl::StrSplit(trailing_element, ", ")) {
                    if (action_index >= state->actions.size()) {
                      matches = false;
                      break;
                    }
                    if (state->actions[action_index] != action) {
                      matches = false;
                      break;
                    }
                    ++action_index;
                  }
                  if (!matches) {
                    state->actions = std::vector<std::string>(
                        absl::StrSplit(trailing_element, ", "));
                  }
                }
              }
              ++trailing_elements_count;
            }
          }
        } else {
          LOG(FATAL) << "Unexpected element, need to consume " << element;
        }
        ++element_number;
      }

      // Validate that everything makes sense and we have the elements expected.
      if (state->interrupt_number != -1) {
        CHECK_EQ(element_number, cpus_ + 2);
      } else {
        // Only these 3 interrupts are known to not be per core.
        if (state->interrupt_name == "Err" || state->interrupt_name == "ERR" ||
            state->interrupt_name == "MIS") {
          if (new_element) {
            CHECK_LE(element_number, cpus_ + 1);
            state->count.resize(element_number - 1);
          } else {
            CHECK_EQ(state->count.size(), element_number - 1);
          }
        } else {
          CHECK_EQ(element_number, cpus_ + 2);
        }
      }

      if (VLOG_IS_ON(1)) {
        if (state->interrupt_number == -1) {
          LOG(INFO) << "IRQ: " << state->interrupt_name;
        } else {
          LOG(INFO) << "IRQ: " << state->interrupt_number;
        }
        for (unsigned int c : state->count) {
          LOG(INFO) << "  " << c;
        }
        if (!state->chip_name.empty()) {
          LOG(INFO) << "chip_name \"" << state->chip_name << "\"";
        }
        if (!state->description.empty()) {
          LOG(INFO) << "description \"" << state->description << "\"";
        }
        if (!state->hwirq.empty()) {
          LOG(INFO) << "hwirq \"" << state->hwirq << "\"";
        }
        if (!state->actions.empty()) {
          for (const std::string &action : state->actions) {
            LOG(INFO) << "  action \"" << action << "\"";
          }
        }
      }
    }

    ++line_number;
  }
}

void InterruptsStatus::Update() {
  ScopedFD fd(open("/proc/interrupts", O_RDONLY));
  size_t so_far = 0;
  while (true) {
    // Keep growing the size of interrupts_content_ until it holds the whole
    // string.
    size_t kStride = 8192;
    if (interrupts_content_.capacity() < so_far + kStride) {
      interrupts_content_.reserve(interrupts_content_.capacity() + kStride);
    }

    interrupts_content_.resize(interrupts_content_.capacity());

    const ssize_t result = read(fd.get(), interrupts_content_.data() + so_far,
                                interrupts_content_.capacity() - so_far);
    PCHECK(result >= 0) << ": reading from /proc/interrupts";
    if (result == 0) {
      break;
    }
    so_far += result;
  }

  interrupts_content_.resize(so_far);
  Update(
      std::string_view(interrupts_content_.data(), interrupts_content_.size()));
}

}  // namespace aos
