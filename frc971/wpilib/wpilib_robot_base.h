#ifndef FRC971_WPILIB_NEWROBOTBASE_H_
#define FRC971_WPILIB_NEWROBOTBASE_H_

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "frc971/wpilib/ahal/RobotBase.h"

namespace frc971 {
namespace wpilib {

class WPILibRobotBase {
 public:
  virtual void Run() = 0;

  // Runs all the loops.
  void RunLoops() {
    // TODO(austin): SIGINT handler calling Exit on all the loops.
    // TODO(austin): RegisterSignalHandler in ShmEventLoop for others.

    ::std::vector<::std::thread> threads;
    for (size_t i = 1; i < loops_.size(); ++i) {
      threads.emplace_back([this, i]() { loops_[i]->Run(); });
    }
    // Save some memory and run the last one in the main thread.
    loops_[0]->Run();

    for (::std::thread &thread : threads) {
      thread.join();
    }

    AOS_LOG(ERROR, "Exiting WPILibRobot\n");

    ::aos::Cleanup();
  }

 protected:
  // Adds a loop to the list of loops to run.
  void AddLoop(::aos::ShmEventLoop *loop) { loops_.push_back(loop); }

 private:
  // List of the event loops to run in RunLoops.
  ::std::vector<::aos::ShmEventLoop *> loops_;
};

#define AOS_ROBOT_CLASS(_ClassName_) \
  START_ROBOT_CLASS(::frc971::wpilib::WPILibAdapterRobot<_ClassName_>)

template <typename T>
class WPILibAdapterRobot : public frc::RobotBase {
 public:
  void StartCompetition() override {
    ::aos::InitNRT(true);

    robot_.Run();
  }

 private:
  T robot_;
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_NEWROBOTBASE_H_
