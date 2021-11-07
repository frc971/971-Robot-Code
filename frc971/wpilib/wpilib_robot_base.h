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

    LOG(ERROR) << "Exiting WPILibRobot";
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
    PCHECK(setuid(0) == 0) << ": Failed to change user to root";
    // Just allow overcommit memory like usual. Various processes map memory
    // they will never use, and the roboRIO doesn't have enough RAM to handle
    // it. This is in here instead of starter.sh because starter.sh doesn't run
    // with permissions on a roboRIO.
    PCHECK(system("echo 0 > /proc/sys/vm/overcommit_memory") == 0);
    PCHECK(system("busybox ps -ef | grep '\\[ktimersoftd/0\\]' | awk '{print "
                  "$1}' | xargs chrt -f -p 70") == 0);
    PCHECK(system("busybox ps -ef | grep '\\[ktimersoftd/1\\]' | awk '{print "
                  "$1}' | xargs chrt -f -p 70") == 0);
    PCHECK(system("busybox ps -ef | grep '\\[irq/54-eth0\\]' | awk '{print "
                  "$1}' | xargs chrt -f -p 17") == 0);

    // Configure throttling so we reserve 5% of the CPU for non-rt work.
    // This makes things significantly more stable when work explodes.
    // This is in here instead of starter.sh for the same reasons, starter is
    // suid and runs as admin, so this actually works.
    PCHECK(system("/sbin/sysctl -w kernel.sched_rt_period_us=1000000") == 0);
    PCHECK(system("/sbin/sysctl -w kernel.sched_rt_runtime_us=950000") == 0);

    robot_.Run();
  }

 private:
  T robot_;
};

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_NEWROBOTBASE_H_
