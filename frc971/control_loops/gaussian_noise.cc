#include "frc971/control_loops/gaussian_noise.h"

namespace frc971 {
namespace control_loops {

GaussianNoise::GaussianNoise(unsigned int seed, double stddev)
    : stddev_(stddev),
      generator_(seed),
      distribution_(0.0, 1.0) {
  // Everything is initialized now.
}

double GaussianNoise::AddNoiseToSample(double sample) {
  return sample + (distribution_(generator_) * stddev_);
}

}  // namespace control_loops
}  // namespace frc971
