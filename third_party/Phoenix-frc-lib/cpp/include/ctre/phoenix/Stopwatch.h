#pragma once

#include <time.h>

namespace ctre {
namespace phoenix {

class Stopwatch {
public:
	void Start();
	unsigned int DurationMs();
	float Duration();
	
private:
	unsigned long _t0 = 0;
	unsigned long _t1 = 0;
	float _scalar = 0.001f / CLOCKS_PER_SEC;
};

}}
