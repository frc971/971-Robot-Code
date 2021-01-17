#pragma once
#include <re.h>

void rawrtc_crc32c_init(void);

uint32_t rawrtc_crc32c(void const* buffer, size_t length);
