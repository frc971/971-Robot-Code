
#ifndef _SENSOR_PROCESSOR_H_
#define _SENSOR_PROCESSOR_H_

// struct maps a single point x to to a value f of x
typedef struct {
	double x;
	double fx;
} Interpolation;

// a set of mapping to use to determine distance
// in inches given a pixel offset
const Interpolation pixel_to_dist[4] = {
	{43.0, 495.0},
	{98.0, 260.0},
	{145.75, 174.0},
	{216.75, 110.0}};
const Interpolation pixel_to_dist640x480[4] = {
	{86.0, 495.0},
	{196.0, 260.0},
	{291.5, 176.0},
	{433.5, 110.0}};

double interpolate(int num_interp_vals,
		const Interpolation *interp, double value);

#endif //_SENSOR_PROCESSOR_H_

