#include <rawrtcdc/sctp_transport.h>

/*
 * Get the corresponding name for an SCTP transport state.
 */
char const* rawrtc_sctp_transport_state_to_name(enum rawrtc_sctp_transport_state const state) {
    switch (state) {
        case RAWRTC_SCTP_TRANSPORT_STATE_NEW:
            return "new";
        case RAWRTC_SCTP_TRANSPORT_STATE_CONNECTING:
            return "connecting";
        case RAWRTC_SCTP_TRANSPORT_STATE_CONNECTED:
            return "connected";
        case RAWRTC_SCTP_TRANSPORT_STATE_CLOSED:
            return "closed";
        default:
            return "???";
    }
}

/*
 * Get the corresponding name for a congestion control algorithm.
 */
char const* rawrtc_sctp_transport_congestion_ctrl_algorithm_to_name(
    enum rawrtc_sctp_transport_congestion_ctrl const algorithm) {
    switch (algorithm) {
        case RAWRTC_SCTP_TRANSPORT_CONGESTION_CTRL_RFC2581:
            return "RFC2581";
        case RAWRTC_SCTP_TRANSPORT_CONGESTION_CTRL_HSTCP:
            return "HSTCP";
        case RAWRTC_SCTP_TRANSPORT_CONGESTION_CTRL_HTCP:
            return "HTCP";
        case RAWRTC_SCTP_TRANSPORT_CONGESTION_CTRL_RTCC:
            return "RTCC";
        default:
            return "???";
    }
}
