#ifndef AOS_COMMON_BYTEORDER_H_
#define AOS_COMMON_BYTEORDER_H_

#ifndef __VXWORKS__
#include <endian.h> // endian(3)
#endif
#include <string.h>
#include <stdint.h>

// Contains functions for converting between host and network byte order for
// things other than 16/32 bit integers (those are provided by byteorder(3)).
// Also gives a nice templated interface to these functions.

namespace aos {
namespace {

template <typename int_type, int_type (*function)(int_type)>
static inline void copier(const void *in, void *out) {
  // Have confirmed by looking at the assembly code that this generates the same
  // code as the (undefined by the c++ standard) way of using a union to do it.
  // (which is pretty efficient)
  int_type temp;
  memcpy(&temp, in, sizeof(int_type));
  temp = function(temp);
  memcpy(out, &temp, sizeof(int_type));
}
template <typename int_type, typename T, int_type (*function)(int_type)>
static inline T memcpier(T in) {
  copier<int_type, function>(&in, &in);
  return in;
}

// Needed because be64toh etc are macros (not that the manpage says
// anything...).
// These are used instead of ntohs etc because gcc didn't inline those.
#ifdef __clang__
// Apparently the macros use "register", and clang doesn't like that.
#define register
#endif
static inline uint64_t _be64toh(uint64_t in) { return be64toh(in); }
static inline uint64_t _htobe64(uint64_t in) { return htobe64(in); }
static inline uint32_t _be32toh(uint32_t in) { return be32toh(in); }
static inline uint32_t _htobe32(uint32_t in) { return htobe32(in); }
static inline uint16_t _be16toh(uint16_t in) { return be16toh(in); }
static inline uint16_t _htobe16(uint16_t in) { return htobe16(in); }
#ifdef __clang__
#undef register
#endif

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
  static inline void copy(const char *in, T *host) {
    copier<uint16_t, _be16toh>(in, host);
  }
};
template<typename T> class do_ntoh<4, T> {
 public:
  static inline T apply(T net) { return memcpier<uint32_t, T, _be32toh>(net); }
  static inline void copy(const char *in, T *host) {
    copier<uint32_t, _be32toh>(in, host);
  }
};
template<typename T> class do_ntoh<8, T> {
 public:
  static inline T apply(T net) { return memcpier<uint64_t, T, _be64toh>(net); }
  static inline void copy(const char *in, T *host) {
    copier<uint64_t, _be64toh>(in, host);
  }
};

template <size_t Size>
struct do_ntoh_int {
  static inline void copy(const char *in, char *host);
};
template <>
struct do_ntoh_int<1> {
  static inline void copy(const char *in, char *host) { host[0] = in[0]; }
};
template <>
struct do_ntoh_int<2> {
  static inline void copy(const char *in, char *host) {
    copier<uint16_t, _be16toh>(in, host);
  }
};
template <>
struct do_ntoh_int<4> {
  static inline void copy(const char *in, char *host) {
    copier<uint32_t, _be32toh>(in, host);
  }
};
template <>
struct do_ntoh_int<8> {
  static inline void copy(const char *in, char *host) {
    copier<uint64_t, _be64toh>(in, host);
  }
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
  static inline void copy(const T *host, char *out) {
    copier<uint16_t, _htobe16>(host, out);
  }
};
template<typename T> class do_hton<4, T> {
 public:
  static inline T apply(T host) { return memcpier<uint32_t, T, _htobe32>(host); }
  static inline void copy(const T *host, char *out) {
    copier<uint32_t, _htobe32>(host, out);
  }
};
template<typename T> class do_hton<8, T> {
 public:
  static inline T apply(T host) { return memcpier<uint64_t, T, _htobe64>(host); }
  static inline void copy(const T *host, char *out) {
    copier<uint64_t, _htobe64>(host, out);
  }
};

template <size_t Size>
struct do_hton_int {
  static inline void copy(const char *host, char *out);
};
template <>
struct do_hton_int<1> {
  static inline void copy(const char *host, char *out) { out[0] = host[0]; }
};
template <>
struct do_hton_int<2> {
  static inline void copy(const char *host, char *out) {
    copier<uint16_t, _htobe16>(host, out);
  }
};
template <>
struct do_hton_int<4> {
  static inline void copy(const char *host, char *out) {
    copier<uint32_t, _htobe32>(host, out);
  }
};
template <>
struct do_hton_int<8> {
  static inline void copy(const char *host, char *out) {
    copier<uint64_t, _htobe64>(host, out);
  }
};

}  // namespace

// Converts T from network to host byte order.
template <typename T>
static inline T ntoh(T net) {
  return do_ntoh<sizeof(net), T>::apply(net);
}
// Converts T from host to network byte order.
template <typename T>
static inline T hton(T host) {
  return do_hton<sizeof(host), T>::apply(host);
}

template <typename T>
static inline void to_host(const char *input, T *host) {
  do_ntoh<sizeof(*host), T>::copy(input, host);
}
template <typename T>
static inline void to_network(const T *host, char *output) {
  do_hton<sizeof(*host), T>::copy(host, output);
}

template <size_t Size>
static inline void to_host(const char *input, char *host) {
  do_ntoh_int<Size>::copy(input, host);
}
template <size_t Size>
static inline void to_network(const char *host, char *output) {
  do_hton_int<Size>::copy(host, output);
}

} // namespace aos

#endif
