#ifndef CTR_EXCLUDE_WPILIB_CLASSES

#include "ctre/phoenix/HsvToRgb.h"
#include <math.h>

namespace ctre {
namespace phoenix {
/**
 * Convert hue/saturation/and value into RGB values
 *
 * @param   hDegrees    Hue in degrees
 * @param   S           Saturation with range of 0 to 1
 * @param   V           Value with range of 0 to 1
 * @param   r           Calculated Red value of RGB
 * @param   g           Calculated Green value of RGB
 * @param   b           Calculated Blue value of RGB
 */
void HsvToRgb::Convert(double hDegrees, double S, double V, float* r, float* g,
		float* b) {
	double R, G, B;
	double H = hDegrees;

	//Handles wrap-around
	if (H < 0) {
		H += 360;
	};
	if (H >= 360) {
		H -= 360;
	};

	if (V <= 0)
		R = G = B = 0;
	else if (S <= 0)
		R = G = B = V;
	else {
		double hf = H / 60.0;
		int i = (int) floor(hf);
		double f = hf - i;
		double pv = V * (1 - S);
		double qv = V * (1 - S * f);
		double tv = V * (1 - S * (1 - f));
		switch (i) {
		//Red is dominant color
		case 0:
			R = V;
			G = tv;
			B = pv;
			break;

			//Green is dominant color
		case 1:
			R = qv;
			G = V;
			B = pv;
			break;
		case 2:
			R = pv;
			G = V;
			B = tv;
			break;

			//Blue is dominant color
		case 3:
			R = pv;
			G = qv;
			B = V;
			break;
		case 4:
			R = tv;
			G = pv;
			B = V;
			break;

			//Red is dominant color
		case 5:
			R = V;
			G = pv;
			B = qv;
			break;

			//Back-up case statements, in case our math is wrong
		case 6:
			R = V;
			G = tv;
			B = pv;
			break;
		case -1:
			R = V;
			G = pv;
			B = qv;
			break;

			//Color is not defined
		default:
			//pretend color is black and white
			R = G = B = V;
			break;
		}
	}
	*r = (float) R;
	*g = (float) G;
	*b = (float) B;
}
}
}
#endif
