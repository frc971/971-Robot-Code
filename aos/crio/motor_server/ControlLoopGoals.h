#ifndef AOS_CRIO_MOTOR_SERVER_CONTROL_LOOP_GOAL_H_
#define AOS_CRIO_MOTOR_SERVER_CONTROL_LOOP_GOAL_H_

#include <vector>
#include <semLib.h>

namespace aos {

// This class is used to keep track of all the control loop goals. It exists
// because of several bugs discovered in the std::map implementation.
class ControlLoopGoals {
 public:
  struct Goal {
    const char *const name;
    const size_t length;
    void (*const zero)();
    void (*const ntoh)(const char *);
    Goal(const char *name, size_t length, void (*zero)(), void (*ntoh)(const char *)) :
        name(name), length(length), zero(zero), ntoh(ntoh) {}
  };

 private:
  std::vector<Goal *> goals_;

 public:
  ControlLoopGoals() {}
  void Add(const char *name, size_t length, void (*zero)(), void (*ntoh)(const char *)) {
    char *storage = new char[10];
    memcpy(storage, name, sizeof(storage));
    goals_.push_back(new Goal(storage, length, zero, ntoh));
  }
  const Goal *Get(const char *name) {
    for (auto it = goals_.begin(); it != goals_.end(); ++it) {
      if (memcmp((*it)->name, name, sizeof((*it)->name)) == 0) {
        return *it;
      }
    }
    return NULL;
  }
};

} // namespace aos

#endif
