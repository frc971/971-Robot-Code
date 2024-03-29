#ifndef CUDA_UTILS_INC_
#define CUDA_UTILS_INC_
#include <cuda_runtime.h>
#include <cstdint>

#ifndef cudaSafeCall
#define cudaSafeCall(call) cudaSafeCallWrapper((call),__FILE__,__LINE__)
#endif

void cudaSafeCallWrapper(cudaError err, const char* file, const int line);

inline __device__ __host__ uint32_t iDivUp( uint32_t a, uint32_t b )  		{ return (a % b != 0) ? (a / b + 1) : (a / b); }

#endif