#ifndef AOS_EVENTS_SIMPLE_CHANNEL_H_
#define AOS_EVENTS_SIMPLE_CHANNEL_H_

#include <string>

#include "aos/configuration_generated.h"

namespace aos {

// Structure used to store both a name and a type and look it up in a map.
struct SimpleChannel {
  SimpleChannel(const Channel *channel);

  std::string name;
  std::string type;

  std::string DebugString() const;

  bool operator==(const SimpleChannel &other) const;
  bool operator<(const SimpleChannel &other) const;
};

}  // namespace aos

#endif  // AOS_EVENTS_SIMPLE_CHANNEL_H_
