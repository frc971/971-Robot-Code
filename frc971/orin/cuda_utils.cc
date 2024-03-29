#include <iostream>
#include "frc971/orin/cuda_utils.h"

void cudaSafeCallWrapper(cudaError err, const char* file, const int line)
{
	if (cudaSuccess != err)
	{
		std::cout << "=================================================" << std::endl <<
		             "CUDA error : " << std::endl <<
                    "\tFile: " << file << std::endl <<
                    "\tLine Number: " << line << std::endl <<
                    "\tReason: " << cudaGetErrorString(err) << std::endl <<
					"=================================================" << std::endl;
		//cudaDeviceReset();
		//exit(EXIT_FAILURE);
	}
}