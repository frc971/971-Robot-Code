#ifndef FRC971_ORIN_GPU_IMAGE_H_
#define FRC971_ORIN_GPU_IMAGE_H_

template <typename T>
struct GpuImage {
  typedef T type;
  T *data;
  size_t rows;
  size_t cols;
  // Step is in elements, not bytes.
  size_t step;
};

#endif  // FRC971_ORIN_GPU_IMAGE_H_
