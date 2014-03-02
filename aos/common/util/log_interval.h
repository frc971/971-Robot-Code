#ifndef AOS_COMMON_UTIL_LOG_INTERVAL_H_
#define AOS_COMMON_UTIL_LOG_INTERVAL_H_

#include "aos/common/time.h"
#include "aos/common/logging/logging.h"

#include <string>

namespace aos {
namespace util {

// A class to help with logging things that happen a lot only occasionally.
//
// Intended use {
//   static LogInterval interval(::aos::time::Time::InSeconds(0.2));
//
//   if (WantToLog()) {
//     interval.WantToLog();
//   }
//   if (interval.ShouldLog()) {
//     LOG(DEBUG, "thingie happened! (%d times)\n", interval.Count());
//   }
// }
class LogInterval {
 public:
  constexpr LogInterval(const ::aos::time::Time &interval)
      : count_(0), interval_(interval), last_done_(0, 0) {}

  void WantToLog() {
    if (count_ == 0) {
      last_done_ = ::aos::time::Time::Now();
    }
    ++count_;
  }
  bool ShouldLog() {
    const ::aos::time::Time now = ::aos::time::Time::Now();
    const bool r = (now - last_done_) >= interval_ && count_ > 0;
    if (r) {
      last_done_ = now;
    }
    return r;
  }
  int Count() {
    const int r = count_;
    count_ = 0;
    return r;
  }

  const ::aos::time::Time &interval() const { return interval_; }

 private:
  int count_;
  const ::aos::time::Time interval_;
  ::aos::time::Time last_done_;
};

// This one is even easier to use. It always logs with a message "%s %d
// times\n". Call LOG_INTERVAL wherever it should log and make sure Print gets
// called often (ie not after a conditional return)
class SimpleLogInterval {
 public:
  SimpleLogInterval(const ::aos::time::Time &interval, log_level level,
                    const ::std::string &message)
      : interval_(interval), level_(level), message_(message) {}

#define LOG_INTERVAL(simple_log) \
  simple_log.WantToLog(LOG_SOURCENAME ": " STRINGIFY(__LINE__))
  void WantToLog(const char *context) {
    context_ = context;
    interval_.WantToLog();
  }

  void Print() {
    if (interval_.ShouldLog()) {
      CHECK_NOTNULL(context_);
      log_do(level_, "%s: %.*s %d times over %f sec\n", context_,
             static_cast<int>(message_.size()), message_.data(),
             interval_.Count(), interval_.interval().ToSeconds());
      context_ = NULL;
    }
  }

 private:
  LogInterval interval_;
  const log_level level_;
  const ::std::string message_;
  const char *context_ = NULL;
};

}  // namespace util
}  // namespace aos

#endif  // AOS_COMMON_UTIL_LOG_INTERVAL_H_
