#include "capabilities.h"
#include <rawrtcdc/sctp_capabilities.h>
#include <rawrtcc/code.h>
#include <re.h>

/*
 * Get the SCTP parameter's maximum message size value.
 *
 * Note: A value of `0` indicates that the implementation supports
 *       receiving messages of arbitrary size.
 */
enum rawrtc_code rawrtc_sctp_capabilities_get_max_message_size(
    uint64_t* const max_message_sizep,  // de-referenced
    struct rawrtc_sctp_capabilities* const capabilities) {
    // Check arguments
    if (!capabilities) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set value
    *max_message_sizep = capabilities->max_message_size;
    return RAWRTC_CODE_SUCCESS;
}
