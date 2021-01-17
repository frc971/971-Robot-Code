#include "transport.h"
#include <rawrtcdc/sctp_redirect_transport.h>
#include <rawrtcc/code.h>
#include <re.h>

/*
 * Get the state of the SCTP redirect transport.
 */
enum rawrtc_code rawrtc_sctp_redirect_transport_get_state(
    enum rawrtc_sctp_redirect_transport_state* const statep,  // de-referenced
    struct rawrtc_sctp_redirect_transport* const transport) {
    // Check arguments
    if (!statep || !transport) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set state & done
    *statep = transport->state;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the redirected local SCTP port of the SCTP redirect transport.
 */
enum rawrtc_code rawrtc_sctp_redirect_transport_get_port(
    uint16_t* const portp,  // de-referenced
    struct rawrtc_sctp_redirect_transport* const transport) {
    // Check arguments
    if (!portp || !transport) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set port & done
    *portp = transport->local_port;
    return RAWRTC_CODE_SUCCESS;
}
