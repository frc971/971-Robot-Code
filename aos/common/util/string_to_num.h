#ifndef AOS_COMMON_UTIL_STRING_TO_NUM_H_
#define AOS_COMMON_UTIL_STRING_TO_NUM_H_

#include <sstream>
#include <string>

namespace aos {
namespace util {

// Converts a string into a specified integral type. If it can't be converted
// completely or at all, or if the converted number would overflow the
// specified integral type, it returns false.
template<typename T>
inline bool StringToInteger(const ::std::string &input, T *out_num) {
  ::std::istringstream stream(input);
  stream >> *out_num;

  if (stream.fail() || !stream.eof()) {
    return false;
  }

  return true;
}


}  // util
}  // aos

#endif
