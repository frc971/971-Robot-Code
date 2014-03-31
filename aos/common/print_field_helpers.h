#ifndef AOS_COMMON_PRINT_FIELD_HELPERS_H_
#define AOS_COMMON_PRINT_FIELD_HELPERS_H_

#include <stdint.h>

#include <type_traits>

namespace aos {

template<typename T>
inline bool PrintInteger(char *buf, T val, size_t *output) {
  static const bool is_signed = ::std::is_signed<T>::value;
  const bool is_negative =
      is_signed ? (val & (static_cast<T>(1) << (sizeof(T) * 8 - 1))) : false;

  size_t len = 0;
  if (is_negative) {
    do {
      if (len == *output) return false;
      buf[len++] = '0' - (val % 10);
      val /= 10;
    } while (val != 0);
    if (len == *output) return false;
    buf[len++] = '-';
  } else {
    do {
      if (len == *output) return false;
      buf[len++] = '0' + (val % 10);
      val /= 10;
    } while (val != 0);
  }
  for (size_t i = 0; i < (len >> 1); i++) {
    std::swap(buf[len - 1 - i], buf[i]);
  }
  *output -= len;
  return true;
}

}  // namespace aos

#endif  // AOS_COMMON_PRINT_FIELD_HELPERS_H_
