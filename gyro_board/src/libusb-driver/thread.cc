#include "thread.h"

#include <boost/bind.hpp>


void Thread::Terminate() {
  boost::lock_guard<boost::mutex> lockguard(terminate_mutex_);
  should_terminate_ = true;
}

bool Thread::should_run() {
  boost::lock_guard<boost::mutex> lockguard(terminate_mutex_);
  return !should_terminate_;
}
