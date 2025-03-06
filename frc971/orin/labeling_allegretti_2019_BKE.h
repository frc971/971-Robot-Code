#ifndef FRC971_ORIN_LABELING_ALLEGRETTI_2019_BKE_H_
#define FRC971_ORIN_LABELING_ALLEGRETTI_2019_BKE_H_

#include <cstdint>
#include <cuda_runtime.h>

#include "frc971/orin/gpu_image.h"

void LabelImage(const GpuImage<uint8_t> input, GpuImage<uint32_t> output,
                GpuImage<uint32_t> union_markers_size_device,
                cudaStream_t stream);

#endif  // FRC971_ORIN_LABELING_ALLEGRETTI_2019_BKE_H_
