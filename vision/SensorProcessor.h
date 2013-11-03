#ifndef VISION_SENSOR_PROCESSOR_H_
#define VISION_SENSOR_PROCESSOR_H_

namespace frc971 {

// struct maps a single point x to to a value f of x
typedef struct {
	double x;
	double fx;
} Interpolation;

static const Interpolation kPixelsToMeters[] = {
  {-0.050781, 4.7498},
  {-0.0375, 4.318},
  {0.028125, 3.9878},
  {0.080469, 3.51},
  {0.126563, 3.1496},
  {0.131, 2.9972},
  {0.144, 2.921},
  {0.196, 3.2258},
  // Below here is junk because it starts coming off of the tower base.
  {0.296875, 2.667},
  {0.351562, 2.3876},
};

// Must be in reverse order in meters.
static const Interpolation kMetersToShooterSpeeds[] = {
  {2.0, 375.0},
  {3.0, 360.0},
  {4.5, 375.0},
};

static const Interpolation kMetersToShooterAngles[] = {
  {3.0, 0.68},
  {3.7, 0.635},
  {4.15, 0.58},
  {5.0, 0.51},
};

double interpolate(int num_interp_vals,
		const Interpolation *interp, double value);

}  // namespace frc971

#endif  // VISION_SENSOR_PROCESSOR_H_
