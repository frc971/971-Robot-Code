#pragma once
#include <re.h>
#include <usrsctp.h>

static inline uint32_t rawrtc_crc32c_software(void const* buffer, size_t length) {
    return usrsctp_crc32c((void*) buffer, length);
}
