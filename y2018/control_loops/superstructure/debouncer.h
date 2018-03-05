#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_DEBOUNCER_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_DEBOUNCER_H_

namespace y2018 {
namespace control_loops {
namespace superstructure {

// Ensures that a certain number of states of a certain type are recieved before
// the actual state is changed.
class Debouncer {
 public:
  // Parameters:
  //  - initial_state: the initial state of the debouncer. (assigned to
  // current_state)
  //  - inputs_before_change: the number of inputs of the same type (true or
  // false) required before the debouncer state is changed.
  Debouncer(bool initial_state, int inputs_before_change)
      : current_state_(initial_state),
        inputs_before_change_(inputs_before_change) {}

  // Updates the debounder state with a new input value.
  void Update(bool new_state);

  // Retrieves the current debouncer state.
  bool current_state() const { return current_state_; }

 private:
  // Stores the current debouncer state.
  bool current_state_;

  // Stores the number of inputs of the same type (true or false) required
  // before the debouncer state changes.
  const int inputs_before_change_;

  // Stores the temporary count of inputs of the same type. When this number
  // reaches inputs_before_change_, the debouncer state changes.
  int consistent_count_ = 0;
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_DEBOUNCER_H_
