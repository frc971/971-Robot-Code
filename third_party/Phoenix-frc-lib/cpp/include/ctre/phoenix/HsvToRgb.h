#pragma once

namespace ctre {
namespace phoenix {

class HsvToRgb {
public:
	static void Convert(double hDegrees, double S, double V, float* r, float* g,
			float* b);
};

}
}
