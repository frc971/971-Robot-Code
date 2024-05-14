#ifndef FRC971_ORIN_THRESHOLD_H_
#define FRC971_ORIN_THRESHOLD_H_

#include <stdint.h>

#include "frc971/orin/apriltag_input_format.h"
#include "frc971/orin/cuda.h"

namespace frc971::apriltag {

// Converts to grayscale, decimates, and thresholds an image on the provided
// stream.
template <InputFormat INPUT_FORMAT>
void CudaToGreyscaleAndDecimateHalide(
    const uint8_t *color_image, uint8_t *gray_image, uint8_t *decimated_image,
    uint8_t *unfiltered_minmax_image, uint8_t *minmax_image,
    uint8_t *thresholded_image, size_t width, size_t height,
    size_t min_white_black_diff, CudaStream *stream);

}  // namespace frc971::apriltag

#endif  // FRC971_ORIN_THRESHOLD_H_
