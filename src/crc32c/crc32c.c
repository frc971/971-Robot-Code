#include "crc32c.h"
#include "software.h"
#include "../main/main.h"
#include <rawrtcdc/config.h>
#include <re.h>

#if RAWRTCDC_ENABLE_SSE42_CRC32C
#    include "sse42.h"
#endif

#define DEBUG_MODULE "crc32c"
//#define RAWRTC_DEBUG_MODULE_LEVEL 7 // Note: Uncomment this to debug this module only
#include <rawrtcc/debug.h>

/*
 * Initialise CRC32-C.
 */
void rawrtc_crc32c_init(void) {
#if RAWRTCDC_ENABLE_SSE42_CRC32C
    if (rawrtc_crc32c_sse42_supported()) {
        rawrtc_crc32c_init_sse42();
        rawrtcdc_global.crc32c_handler = rawrtc_crc32c_sse42;
        DEBUG_PRINTF("Initialised CRC32-C (sse42)\n");
        return;
    }
#endif
    rawrtcdc_global.crc32c_handler = rawrtc_crc32c_software;
    DEBUG_PRINTF("Initialised CRC32-C (software)\n");
}

/*
 * Compute CRC-32C using whatever method has been established.
 */
uint32_t rawrtc_crc32c(void const* buffer, size_t length) {
    return rawrtcdc_global.crc32c_handler(buffer, length);
}
