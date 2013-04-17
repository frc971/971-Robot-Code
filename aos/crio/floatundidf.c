#include <stdint.h>

// stolen from
// http://llvm.org/svn/llvm-project/compiler-rt/trunk/lib/floatundidf.c@179666
//
// Lame hack to provide a libgcc function that the version on the crio doesn't
// have.
// TODO(brians): make this nicer and figure out which other ones we need

double __floatundidf(unsigned long long a)
{
	static const double twop52 = 0x1.0p52;
	static const double twop84 = 0x1.0p84;
	static const double twop84_plus_twop52 = 0x1.00000001p84;
	
	union { uint64_t x; double d; } high = { .d = twop84 };
	union { uint64_t x; double d; } low = { .d = twop52 };
	
	high.x |= a >> 32;
	low.x |= a & 0x00000000ffffffffULL;
	
	const double result = (high.d - twop84_plus_twop52) + low.d;
	return result;
}
