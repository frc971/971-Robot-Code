#ifndef FRC971_CONTROL_LOOPS_GAUSSIAN_NOISE_H_
#define FRC971_CONTROL_LOOPS_GAUSSIAN_NOISE_H_

#include <unistd.h>

#include <memory>
#include <random>

namespace frc971 {
namespace control_loops {

class GaussianNoise {
 public:
  // seed: The seed for the random number generator.
  // stddev: The standard deviation of the distribution.
  GaussianNoise(unsigned int seed, double stddev);

  // Returns a version of the sample with gaussian noise added in.
  double AddNoiseToSample(double sample);

  // Sets the standard deviation of the gaussian noise.
  inline void set_standard_deviation(double stddev) {
    stddev_ = stddev;
  }

 private:
  double stddev_;

  std::default_random_engine generator_;
  std::normal_distribution<double> distribution_;
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_GAUSSIAN_NOISE_H_
