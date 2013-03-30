#include "vision/SensorProcessor.h"

#include <stdio.h>

namespace frc971 {

// give a set of x -> fx pairs find a range for our value
// then interpolate between and return an interpolated fx.
// If the value is off the end just extend the line to
// meet our point. If something does go wrong (and it
// never should) it will return -1.
double interpolate(int num_interp_vals,
		const Interpolation *interp, double value) {
	double dy;
	double dx;
	double a;
	double intercept;
	//printf("for val %.1f\n", value); 
	if (value < interp[0].x) {
		// if closer than nearest 
		dy = interp[1].fx - interp[0].fx;
		dx = interp[1].x - interp[0].x;
		a = value - interp[0].x;
		intercept = interp[0].fx;
		//printf("LESS THAN\n");
	} else if (value > interp[num_interp_vals-1].x){
		// if further than furthest 
		dy = interp[num_interp_vals-1].fx - interp[num_interp_vals-2].fx;
		dx = interp[num_interp_vals-1].x - interp[num_interp_vals-2].x;
		a = value - interp[num_interp_vals-2].x;
		intercept = interp[num_interp_vals-2].fx;
		//printf("GT THAN\n");
	} else {
		//printf("gh0\n");
		// scan for range
		for(int i=0; i<num_interp_vals-1; i++){
			if(value >= interp[i].x && value <= interp[i+1].x){
		//		printf("(%.1f,%.1f)=(%.1f,%.1f)\n",
		//				interp[i].x, interp[i+1].x,
		//				interp[i].fx, interp[i+1].fx);
				double lambda =
					(value - interp[i].x)/(interp[i+1].x - interp[i].x);
				return (1-lambda)*interp[i].fx + lambda*interp[i+1].fx;
			}
		}
		// this should maybe be an assert
		return -1;
	}
	
	return ( (dy/dx)*a + intercept );
}

}  // namespace frc971
