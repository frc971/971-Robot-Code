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

// Must be in reverse order in meters.
static const Interpolation kMetersToShooterSpeeds[] = {
  {12.5, 375.0},
  {21.0, 360.0},
  {25.0, 375.0},
};

static const Interpolation kMetersToShooterAngles[] = {
  {0.0, 0.7267},
  {12.5, 0.7267},
  {16.5, 0.604},
  {18.0, 0.587},
  {20.0, 0.576},
  {21.0, 0.550},
  {23.0, 0.540},
};

double interpolate(int num_interp_vals,
		const Interpolation *interp, double value);

}  // namespace frc971

#endif  // VISION_SENSOR_PROCESSOR_H_
