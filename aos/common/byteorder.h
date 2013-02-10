#ifndef AOS_COMMON_BYTEORDER_H_
#define AOS_COMMON_BYTEORDER_H_

#ifndef __VXWORKS__
#include <endian.h> // endian(3)
#endif

// Contains functions for converting between host and network byte order for
// things other than 16/32 bit integers (those are provided by byteorder(3)).
// Also gives a nice templated interface to these functions.

namespace aos {

#ifndef __VXWORKS__
namespace {

template<typename int_type, typename T, int_type (*function)(int_type)> static inline void copier(const void *in, void *out) {
  static_assert(sizeof(T) == sizeof(int_type), "bad template args");
  // Have confirmed by looking at the assembly code that this generates the same
  // code as the (undefined by the c++ standard) way of using a union to do it.
  // (which is pretty efficient)
  int_type temp;
  memcpy(&temp, in, sizeof(T));
  temp = function(temp);
  memcpy(out, &temp, sizeof(T));
}
template<typename int_type, typename T, int_type (*function)(int_type)> static inline T memcpier(T in) {
  copier<int_type, T, function>(&in, &in);
  return in;
}

// Can't make them static because they have to have external linkage to be used as a
// template parameter.
// Needed because be64toh etc are macros. (not that the manpage says anything...)
// These are used instead of ntohs etc because gcc didn't inline those.
inline uint64_t _be64toh(uint64_t in) { return be64toh(in); }
inline uint64_t _htobe64(uint64_t in) { return htobe64(in); }
inline uint32_t _be32toh(uint32_t in) { return be32toh(in); }
inline uint32_t _htobe32(uint32_t in) { return htobe32(in); }
inline uint16_t _be16toh(uint16_t in) { return be16toh(in); }
inline uint16_t _htobe16(uint16_t in) { return htobe16(in); }

template<int bytes, typename T> class do_ntoh {
 public:
  static inline T apply(T net);
  static inline void copy(const char *in, T &net);
};
template<typename T> class do_ntoh<1, T> {
 public:
  static inline T apply(T net) { return net; }
  static inline void copy(const char *in, T *host) { host[0] = in[0]; }
};
template<typename T> class do_ntoh<2, T> {
 public:
  static inline T apply(T net) { return memcpier<uint16_t, T, _be16toh>(net); }
  static inline void copy(const char *in, T *host) { copier<uint16_t, T, _be16toh>(in, host); }
};
template<typename T> class do_ntoh<4, T> {
 public:
  static inline T apply(T net) { return memcpier<uint32_t, T, _be32toh>(net); }
  static inline void copy(const char *in, T *host) { copier<uint32_t, T, _be32toh>(in, host); }
};
template<typename T> class do_ntoh<8, T> {
 public:
  static inline T apply(T net) { return memcpier<uint64_t, T, _be64toh>(net); }
  static inline void copy(const char *in, T *host) { copier<uint64_t, T, _be64toh>(in, host); }
};
template<int bytes, typename T> class do_hton {
 public:
  static inline T apply(T host);
  static inline void copy(const T *host, char *out);
};
template<typename T> class do_hton<1, T> {
 public:
  static inline T apply(T host) { return host; }
  static inline void copy(const T *host, char *out) { out[0] = host[0]; }
};
template<typename T> class do_hton<2, T> {
 public:
  static inline T apply(T host) { return memcpier<uint16_t, T, _htobe16>(host); }
  static inline void copy(const T *host, char *out) { copier<uint16_t, T, _htobe16>(host, out); }
};
template<typename T> class do_hton<4, T> {
 public:
  static inline T apply(T host) { return memcpier<uint32_t, T, _htobe32>(host); }
  static inline void copy(const T *host, char *out) { copier<uint32_t, T, _htobe32>(host, out); }
};
template<typename T> class do_hton<8, T> {
 public:
  static inline T apply(T host) { return memcpier<uint64_t, T, _htobe64>(host); }
  static inline void copy(const T *host, char *out) { copier<uint64_t, T, _htobe64>(host, out); }
};

} // namespace
#endif // ifndef __VXWORKS__

// Converts T from network to host byte order.
template<typename T> static inline T ntoh(T net) {
#ifndef __VXWORKS__
  return do_ntoh<sizeof(net), T>::apply(net);
#else
  return net;
#endif
}
// Converts T from host to network byte order.
template<typename T> static inline T hton(T host) {
#ifndef __VXWORKS__
  return do_hton<sizeof(host), T>::apply(host);
#else
  return host;
#endif
}

template<typename T> static inline void to_host(const char *input, T *host) {
#ifndef __VXWORKS__
  do_ntoh<sizeof(*host), T>::copy(input, host);
#else
  memcpy(host, input, sizeof(*host));
#endif
}
template<typename T> static inline void to_network(const T *host, char *output) {
#ifndef __VXWORKS__
  do_hton<sizeof(*host), T>::copy(host, output);
#else
  memcpy(output, host, sizeof(*host));
#endif
}

} // namespace aos

#endif

