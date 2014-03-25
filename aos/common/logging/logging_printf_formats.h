#ifndef AOS_COMMON_LOGGING_LOGGING_PRINTF_FORMATS_H_
#define AOS_COMMON_LOGGING_LOGGING_PRINTF_FORMATS_H_

#include "aos/common/macros.h"

// This file has printf(3) formats and corresponding arguments for printing out
// times and log messages.
// They are all split out as macros because there are 2 things that want to
// print using the same format: log_displayer and PrintMessage in
// logging_impl.cc.

#define AOS_TIME_FORMAT \
  "%010" PRId32 ".%0" STRINGIFY(AOS_TIME_NSECONDS_DIGITS) PRId32 "s"
#define AOS_TIME_ARGS(sec, nsec)                      \
  sec, (nsec + (AOS_TIME_NSECONDS_DENOMINATOR / 2)) / \
      AOS_TIME_NSECONDS_DENOMINATOR

#define AOS_LOGGING_BASE_FORMAT \
  "%.*s(%" PRId32 ")(%05" PRIu16 "): %-7s at " AOS_TIME_FORMAT ": "
#define AOS_LOGGING_BASE_ARGS(name_length, name, source, sequence, level, sec, \
                              nsec)                                            \
  static_cast<int>(name_length), name, source, sequence,                       \
      ::aos::logging::log_str(level), AOS_TIME_ARGS(sec, nsec)

// These 2 define how many digits we use to print out the nseconds fields of
// times. They have to stay matching.
#define AOS_TIME_NSECONDS_DIGITS 5
#define AOS_TIME_NSECONDS_DENOMINATOR 10000

#endif  // AOS_COMMON_LOGGING_LOGGING_PRINTF_FORMATS_H_
