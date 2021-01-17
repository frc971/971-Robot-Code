#include "main.h"
#include "../crc32c/crc32c.h"
#include <rawrtcdc/main.h>
#include <rawrtcc/code.h>
#include <re.h>
#include <usrsctp.h>  // usrsctp_handle_timers
#include <limits.h>  // INT_MAX

/*
 * Global RAWRTCDC settings.
 */
struct rawrtcdc_global rawrtcdc_global;

/*
 * Initialise RAWRTCDC. Must be called before making a call to any
 * other function.
 *
 * Note: In case `init_re` is not set to `true`, you MUST initialise
 *       re yourselves before calling this function.
 */
enum rawrtc_code rawrtcdc_init(bool const init_re, rawrtcdc_timer_handler const timer_handler) {
    // Check arguments
    if (!timer_handler) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Initialise re (if requested)
    if (init_re) {
        if (libre_init()) {
            return RAWRTC_CODE_INITIALISE_FAIL;
        }
    }

    // Initialise CRC32-C
    rawrtc_crc32c_init();

    // Set timer handler
    rawrtcdc_global.timer_handler = timer_handler;

    // Set usrsctp initialised counter
    rawrtcdc_global.usrsctp_initialized = 0;

    // Done
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Close RAWRTCDC and free up all resources.
 *
 * Note: In case `close_re` is not set to `true`, you MUST close
 *       re yourselves.
 */
enum rawrtc_code rawrtcdc_close(bool const close_re) {
    // TODO: Close usrsctp if initialised

    // Remove timer handler
    rawrtcdc_global.timer_handler = NULL;

    // Close re (if requested)
    if (close_re) {
        libre_close();
    }

    // Done
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Handle timer tick.
 * `delta` contains the delta milliseconds passed between calls.
 */
void rawrtcdc_timer_tick(uint_fast16_t const delta) {
    // Pass delta ms to usrsctp
#if (UINT16_MAX > INT_MAX)
    usrsctp_handle_timers(delta > INT_MAX ? INT_MAX : ((int) delta));
#else
    usrsctp_handle_timers((int) delta);
#endif
}
