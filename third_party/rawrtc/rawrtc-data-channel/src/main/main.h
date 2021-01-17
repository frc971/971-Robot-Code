#pragma once
#include <rawrtcdc/main.h>
#include <re.h>

extern struct rawrtcdc_global rawrtcdc_global;

/*
 * CRC32-C handler to be used.
 */
typedef uint32_t (*rawrtc_crc32c_handler)(void const* buffer, size_t length);

/*
 * Global RAWRTCDC settings.
 */
struct rawrtcdc_global {
    rawrtcdc_timer_handler timer_handler;
    rawrtc_crc32c_handler crc32c_handler;
    uint_fast32_t usrsctp_initialized;
    size_t usrsctp_chunk_size;
};
