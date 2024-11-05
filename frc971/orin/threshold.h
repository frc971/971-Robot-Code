#ifndef FRC971_ORIN_THRESHOLD_H_
#define FRC971_ORIN_THRESHOLD_H_

#include <stdint.h>

#include "frc971/orin/apriltag_input_format.h"
#include "frc971/orin/cuda.h"

namespace frc971::apriltag {

class BaseThreshold
{
public:
    // Create a full-size grayscale image from a color image on the provided stream.
    virtual void CudaToGreyscale(const uint8_t *color_image, uint8_t *gray_image,
                        uint32_t width, uint32_t height, CudaStream *stream) = 0;

    // Converts to grayscale, decimates, and thresholds an image on the provided
    // stream.
    virtual void CudaToGreyscaleAndDecimateHalide(
        const uint8_t *color_image, uint8_t *gray_image, uint8_t *decimated_image,
        uint8_t *unfiltered_minmax_image, uint8_t *minmax_image,
        uint8_t *thresholded_image, uint32_t width, uint32_t height,
        uint32_t min_white_black_diff, CudaStream *stream) = 0;

    virtual ~BaseThreshold() = default;
}; // class BaseThreshold

template <InputFormat INPUT_FORMAT>
class Threshold : public BaseThreshold
{
public:
    // Create a full-size grayscale image from a color image on the provided stream.
    void CudaToGreyscale(const uint8_t *color_image, uint8_t *gray_image,
                        uint32_t width, uint32_t height, CudaStream *stream) override;

    // Converts to grayscale, decimates, and thresholds an image on the provided
    // stream.
    void CudaToGreyscaleAndDecimateHalide(
        const uint8_t *color_image, uint8_t *gray_image, uint8_t *decimated_image,
        uint8_t *unfiltered_minmax_image, uint8_t *minmax_image,
        uint8_t *thresholded_image, uint32_t width, uint32_t height,
        uint32_t min_white_black_diff, CudaStream *stream) override;

    virtual ~Threshold() = default;
}; // class Threshold

}  // namespace frc971::apriltag

#endif  // FRC971_ORIN_THRESHOLD_H_
