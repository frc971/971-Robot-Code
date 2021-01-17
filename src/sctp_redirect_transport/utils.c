#include <rawrtcdc/sctp_redirect_transport.h>

/*
 * Get the corresponding name for an SCTP redirect transport state.
 */
char const* rawrtc_sctp_redirect_transport_state_to_name(
    enum rawrtc_sctp_redirect_transport_state const state) {
    switch (state) {
        case RAWRTC_SCTP_REDIRECT_TRANSPORT_STATE_NEW:
            return "new";
        case RAWRTC_SCTP_REDIRECT_TRANSPORT_STATE_OPEN:
            return "open";
        case RAWRTC_SCTP_REDIRECT_TRANSPORT_STATE_CLOSED:
            return "closed";
        default:
            return "???";
    }
}
