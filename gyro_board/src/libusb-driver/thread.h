#ifndef THREAD_H_
#define THREAD_H_

#include <sys/socket.h>
#include <linux/can.h>
#include <boost/thread/locks.hpp>
#include <boost/thread.hpp>

class Thread {
 public:
  // Constructer initializes terminate to false.
  Thread() : should_terminate_(false) {}

  // Terminate the thread soon.
  void Terminate();

 protected:
  // Helper method to atomically read the should_terminate_ boolean.
  bool should_run();

 private:
  // Stores whether or not the thread has been asked to quit.
  // TODO(aschuh): This really should be a Notification...
  bool should_terminate_;
  // Mutex to protect should_terminate.
  boost::mutex terminate_mutex_;
};

#endif  // THREAD_H_
