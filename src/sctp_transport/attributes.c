#include "transport.h"
#include <rawrtcdc/data_channel.h>
#include <rawrtcdc/sctp_capabilities.h>
#include <rawrtcdc/sctp_transport.h>
#include <re.h>

/*
 * Get the current state of the SCTP transport.
 */
enum rawrtc_code rawrtc_sctp_transport_get_state(
    enum rawrtc_sctp_transport_state* const statep,  // de-referenced
    struct rawrtc_sctp_transport* const transport) {
    // Check arguments
    if (!statep || !transport) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set state & done
    *statep = transport->state;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the local port of the SCTP transport.
 */
enum rawrtc_code rawrtc_sctp_transport_get_port(
    uint16_t* const portp,  // de-referenced
    struct rawrtc_sctp_transport* const transport) {
    // Check arguments
    if (!portp || !transport) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set port & done
    *portp = transport->port;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the number of streams allocated for the SCTP transport.
 */
enum rawrtc_code rawrtc_sctp_transport_get_n_streams(
    uint16_t* const n_streamsp,  // de-referenced
    struct rawrtc_sctp_transport* const transport) {
    // Check arguments
    if (!n_streamsp || !transport) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set #streams & done
    *n_streamsp = (uint16_t) transport->n_channels;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the local SCTP transport capabilities (static).
 * `*capabilitiesp` must be unreferenced.
 */
enum rawrtc_code rawrtc_sctp_transport_get_capabilities(
    struct rawrtc_sctp_capabilities** const capabilitiesp  // de-referenced
) {
    return rawrtc_sctp_capabilities_create(capabilitiesp, RAWRTC_SCTP_TRANSPORT_MAX_MESSAGE_SIZE);
}

/*
 * Set the SCTP transport's data channel handler.
 */
enum rawrtc_code rawrtc_sctp_transport_set_data_channel_handler(
    struct rawrtc_sctp_transport* const transport,
    rawrtc_data_channel_handler const data_channel_handler  // nullable
) {
    // Check arguments
    if (!transport) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set data channel handler & done
    transport->data_channel_handler = data_channel_handler;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the SCTP transport's data channel handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_sctp_transport_get_data_channel_handler(
    rawrtc_data_channel_handler* const data_channel_handlerp,  // de-referenced
    struct rawrtc_sctp_transport* const transport) {
    // Check arguments
    if (!data_channel_handlerp || !transport) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Get message handler (if any)
    if (transport->data_channel_handler) {
        *data_channel_handlerp = transport->data_channel_handler;
        return RAWRTC_CODE_SUCCESS;
    } else {
        return RAWRTC_CODE_NO_VALUE;
    }
}

/*
 * Set the SCTP transport's state change handler.
 */
enum rawrtc_code rawrtc_sctp_transport_set_state_change_handler(
    struct rawrtc_sctp_transport* const transport,
    rawrtc_sctp_transport_state_change_handler const state_change_handler  // nullable
) {
    // Check arguments
    if (!transport) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set data channel handler & done
    transport->state_change_handler = state_change_handler;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the SCTP transport's state change handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_sctp_transport_get_state_change_handler(
    rawrtc_sctp_transport_state_change_handler* const state_change_handlerp,  // de-referenced
    struct rawrtc_sctp_transport* const transport) {
    // Check arguments
    if (!state_change_handlerp || !transport) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Get message handler (if any)
    if (transport->state_change_handler) {
        *state_change_handlerp = transport->state_change_handler;
        return RAWRTC_CODE_SUCCESS;
    } else {
        return RAWRTC_CODE_NO_VALUE;
    }
}
