#ifndef __COMMON_SENSOR_VALUES_H_
#define __COMMON_SENSOR_VALUES_H_

#include <stdint.h>

namespace frc971 {

struct sensor_values {
	union {
		struct {
			int32_t lencoder, rencoder;
		};
		uint32_t encoders[2];
	};

  // TODO(2013) all the rest
};

} // namespace frc971

#endif

