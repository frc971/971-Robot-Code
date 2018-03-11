#include "ctre/phoenix/Stopwatch.h"

namespace ctre {
namespace phoenix {

void Stopwatch::Start(){
	_t0 = (float)clock();
}
unsigned int Stopwatch::DurationMs(){
	return Stopwatch::Duration() * 1000;
}
float Stopwatch::Duration(){
	_t1 = (float)clock();
	long retval = _t1 - _t0;
	if(retval < 0) retval = 0;
	return retval * _scalar;
}

} // namespace phoenix
}
