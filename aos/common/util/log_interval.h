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
//   static LogInterval interval(::std::chrono::millseconds(200));
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
  constexpr LogInterval(::std::chrono::nanoseconds interval)
      : interval_(interval) {}

  void WantToLog() {
    if (count_ == 0) {
      last_done_ = ::aos::monotonic_clock::now();
    }
    ++count_;
  }
  bool ShouldLog() {
    const ::aos::monotonic_clock::time_point now =
        ::aos::monotonic_clock::now();
    const bool r = now >= interval_ + last_done_ && count_ > 0;
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

  ::std::chrono::nanoseconds interval() const { return interval_; }

 private:
  int count_ = 0;
  const ::std::chrono::nanoseconds interval_;
  ::aos::monotonic_clock::time_point last_done_ =
      ::aos::monotonic_clock::min_time;
};

// This one is even easier to use. It always logs with a message "%s %d
// times\n". Call LOG_INTERVAL wherever it should log and make sure Print gets
// called often (ie not after a conditional return)
class SimpleLogInterval {
 public:
  SimpleLogInterval(::std::chrono::nanoseconds interval, log_level level,
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
             interval_.Count(),
             ::std::chrono::duration_cast<::std::chrono::duration<double>>(
                 interval_.interval()).count());
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
