#pragma once

#include <stdint.h>

struct ResetStats
{
	int32_t resetCount;
	int32_t resetFlags;
	int32_t firmVers;
	bool hasReset;
	ResetStats() {
		resetCount = 0;
		resetFlags = 0;
		firmVers = 0;
		hasReset = false;
	}
};
