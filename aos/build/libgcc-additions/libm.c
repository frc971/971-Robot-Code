// Some of the Compiler-RT implementations we use want to use functions from
// libm. However, GCC provides builtins for them, so they don't need to show up
// in <math.h> Unfortunately, those GCC builtins seem to (sometimes, at least)
// just emit calls to the functions themselves, so we have our own
// implementations here. They might not be as fast as they could be, but they
// are correct as far as I can tell.

#include <stdint.h>

double fmax(double a, double b) {
  if (__builtin_isnan(a)) return b;
  if (__builtin_isnan(b)) return a;
  if (a > b) return a;
  return b;
}
float fmaxf(float a, float b) {
  if (__builtin_isnan(a)) return b;
  if (__builtin_isnan(b)) return a;
  if (a > b) return a;
  return b;
}

double scalbn(double x, int exp) {
  return x * (2 << exp);
}
float scalbnf(float x, int exp) {
  return x * (2 << exp);
}

float logbf(float x) {
  union {
    float f;
    int32_t d;
  } converter;
  converter.f = x;
  int32_t ix = converter.d & 0x7fffffff;
  int32_t rix;

  if (ix == 0) {
    return (float)-1.0 / __builtin_fabsf(x);
  } else if (ix >= 0x7f800000) {
    return x * x;
  } else if (__builtin_expect((rix = ix >> 23) == 0, 0)) {
    rix -= __builtin_clz(ix) - 9;
  }
  return (float)(rix - 127);
}
