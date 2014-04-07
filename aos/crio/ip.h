#ifndef AOS_CRIO_IP_H_
#define AOS_CRIO_IP_H_

#include <inetLib.h>
#include <stdint.h>

namespace aos {
namespace util {

// Retrieves the IP address of this cRIO and stores it in *address.
// Loops infinitely until it succeeds.
in_addr GetOwnIPAddress();

}  // namespace util
}  // namespace aos

#endif  // AOS_CRIO_IP_H_
