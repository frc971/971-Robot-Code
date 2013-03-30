#ifndef VISION_SENSOR_PROCESSOR_H_
#define VISION_SENSOR_PROCESSOR_H_

namespace frc971 {

// struct maps a single point x to to a value f of x
typedef struct {
	double x;
	double fx;
} Interpolation;

static const Interpolation kPixelsToMeters[] = {
	{43.0 / 320.0, 12.573},
	{98.0 / 320.0, 6.604},
	{145.75 / 320.0, 4.420},
	{216.75 / 320.0, 2.794},
};

static const Interpolation kMetersToShooterSpeeds[] = {
  {10.0, 200.0},
  {5.0, 175.0},
};

static const Interpolation kMetersToShooterAngles[] = {
  {10.0, 0.7},
  {5.0, 0.9},
};

double interpolate(int num_interp_vals,
		const Interpolation *interp, double value);

}  // namespace frc971

#endif  // VISION_SENSOR_PROCESSOR_H_
