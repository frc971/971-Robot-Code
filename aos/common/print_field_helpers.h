#ifndef AOS_COMMON_PRINT_FIELD_HELPERS_H_
#define AOS_COMMON_PRINT_FIELD_HELPERS_H_

#include <stdint.h>

#include <type_traits>

namespace aos {

template<typename T>
inline bool PrintInteger(char *buf, T val, size_t *output) {
  static const bool is_signed = ::std::is_signed<T>::value;

  size_t len = 0;
  if (is_signed && val <= 0) {
    while (*output >= len && (val != 0 || len == 0)) {
      buf[len++] = '0' - (val % 10);
      val /= 10;
    }
    buf[len++] = '-';
  } else {
    while (*output >= len && (val != 0 || len == 0)) {
      buf[len++] = '0' + (val % 10);
      val /= 10;
    }
  }
  // If we have enough space.
  if (*output >= len) {
    for (size_t i = 0; i < (len >> 1); i++) {
      std::swap(buf[len - 1 - i], buf[i]);
    }
    *output -= len;
    return true;
  } else {
    return false;
  }
}

}  // namespace aos

#endif  // AOS_COMMON_PRINT_FIELD_HELPERS_H_
