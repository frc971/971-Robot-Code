#pragma once
#include <rawrtcc/code.h>
#include <re.h>

// Dependencies
struct rawrtc_sctp_capabilities;
struct rawrtc_sctp_transport_context;

/**
 * SCTP redirect transport states.
 */
enum rawrtc_sctp_redirect_transport_state {
    RAWRTC_SCTP_REDIRECT_TRANSPORT_STATE_NEW,
    RAWRTC_SCTP_REDIRECT_TRANSPORT_STATE_OPEN,
    RAWRTC_SCTP_REDIRECT_TRANSPORT_STATE_CLOSED,
};

/**
 * Redirect transport.
 */
struct rawrtc_sctp_redirect_transport;

/**
 * SCTP redirect transport state change handler.
 */
typedef void (*rawrtc_sctp_redirect_transport_state_change_handler)(
    enum rawrtc_sctp_redirect_transport_state const state, void* const arg);

/**
 * Create an SCTP redirect transport from an external DTLS transport.
 * `*transportp` must be unreferenced.
 *
 * `port` defaults to `5000` if set to `0`.
 * `redirect_ip` is the target IP SCTP packets will be redirected to
 *  and must be a IPv4 address.
 * `redirect_port` is the target SCTP port packets will be redirected
 *  to.
 *
 * Note: The underlying DTLS transport is supposed to be immediately
 *       attached after creation of this transport.
 * Important: The redirect transport requires to be run inside re's
 *            event loop (`re_main`).
 */
enum rawrtc_code rawrtc_sctp_redirect_transport_create_from_external(
    struct rawrtc_sctp_redirect_transport** const transportp,  // de-referenced
    struct rawrtc_sctp_transport_context* const context,  // copied
    uint16_t const port,  // zeroable
    char* const redirect_ip,  // copied
    uint16_t const redirect_port,
    rawrtc_sctp_redirect_transport_state_change_handler const state_change_handler,  // nullable
    void* const arg  // nullable
);

/**
 * Start an SCTP redirect transport.
 */
enum rawrtc_code rawrtc_sctp_redirect_transport_start(
    struct rawrtc_sctp_redirect_transport* const transport,
    struct rawrtc_sctp_capabilities const* const remote_capabilities,  // copied
    uint16_t remote_port  // zeroable
);

/**
 * Stop and close the SCTP redirect transport.
 */
enum rawrtc_code rawrtc_sctp_redirect_transport_stop(
    struct rawrtc_sctp_redirect_transport* const transport);

/**
 * Feed inbound data to the SCTP redirect transport (that will be sent
 * out via the raw socket).
 *
 * `buffer` contains the data to be fed to the raw transport. Since
 * the data is not going to be referenced, you can pass a *fake* `mbuf`
 * structure that hasn't been allocated with `mbuf_alloc` to avoid
 * copying.
 *
 * Return `RAWRTC_CODE_INVALID_STATE` in case the transport is closed.
 * In case the buffer could not be sent due to the raw socket's buffer
 * being too full, `RAWRTC_CODE_TRY_AGAIN_LATER` will be returned. You
 * can safely ignore this code since SCTP will retransmit data on a
 * reliable stream.
 * Otherwise, `RAWRTC_CODE_SUCCESS` is being returned.
 */
enum rawrtc_code rawrtc_sctp_redirect_transport_feed_inbound(
    struct rawrtc_sctp_redirect_transport* const transport, struct mbuf* const buffer);

/**
 * Get the current state of the SCTP redirect transport.
 */
enum rawrtc_code rawrtc_sctp_redirect_transport_get_state(
    enum rawrtc_sctp_redirect_transport_state* const statep,  // de-referenced
    struct rawrtc_sctp_redirect_transport* const transport);

/**
 * Get the redirected local SCTP port of the SCTP redirect transport.
 */
enum rawrtc_code rawrtc_sctp_redirect_transport_get_port(
    uint16_t* const portp,  // de-referenced
    struct rawrtc_sctp_redirect_transport* const transport);

/**
 * Get the corresponding name for an SCTP redirect transport state.
 */
char const* rawrtc_sctp_redirect_transport_state_to_name(
    enum rawrtc_sctp_redirect_transport_state const state);
