#ifndef AOS_STARTER_IRQ_AFFINITY_LIB_H_
#define AOS_STARTER_IRQ_AFFINITY_LIB_H_

#include <string>
#include <vector>

#include "glog/logging.h"

namespace aos {

// Class to parse /proc/interrupts.
class InterruptsStatus {
 public:
  InterruptsStatus();

  // Updates the interrupt state.
  void Update();

  // Updates the interrupt state from the contents of /proc/interrupts.
  //
  // This should only be used for testing.
  void Update(std::string_view contents);

  // Information about each interrupt.
  struct InterruptState {
    // IRQ number.  -1 if this doesn't have a number.
    int interrupt_number;
    // Name of the interrupt.  Only populated when number == -1
    std::string interrupt_name;
    // IRQs triggered per core, where the vector index is the core.
    std::vector<unsigned int> count;

    // The name of the irq chip controller.
    std::string chip_name;

    // Description of the IRQ if it doesn't have an interrupt number.
    std::string description;

    // Hardware IRQ "number".
    std::string hwirq;

    // List of actions.  An action is something which gets triggered on an
    // interrupt.  This is the IRQ "name", and is a vector to cover shared IRQs.
    std::vector<std::string> actions;
  };

  // Information about all IRQs.
  const std::vector<InterruptState> &states() const { return states_; }

 private:

  // Buffer to hold the contents of /proc/interrupts to avoid re-allocating
  // continually.
  std::vector<char> interrupts_content_;

  size_t cpus_ = 0;

  std::vector<InterruptState> states_;
};

}  // namespace aos

#endif  // AOS_STARTER_IRQ_AFFINITY_LIB_H_
