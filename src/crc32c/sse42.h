#pragma once
#include <re.h>

bool rawrtc_crc32c_sse42_supported(void);

void rawrtc_crc32c_init_sse42(void);

uint32_t rawrtc_crc32c_sse42(void const* buffer, size_t length);
