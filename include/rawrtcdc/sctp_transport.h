#pragma once
#include "data_channel.h"
#include <rawrtcc/code.h>
#include <re.h>

// Dependencies
struct rawrtc_data_transport;
struct rawrtc_sctp_capabilities;
struct rawrtc_sctp_transport_context;

/**
 * Missing in usrsctp.h
 */
#define SCTP_PLUGGABLE_CC 0x00001202

/**
 * SCTP transport checksum configuration flags.
 * TODO: Add configuration to make these applyable.
 */
enum rawrtc_sctp_transport_checksum_flags {
    RAWRTC_SCTP_TRANSPORT_CHECKSUM_ENABLE_ALL = 0,
    RAWRTC_SCTP_TRANSPORT_CHECKSUM_DISABLE_INBOUND = 1 << 0,
    RAWRTC_SCTP_TRANSPORT_CHECKSUM_DISABLE_OUTBOUND = 1 << 1,
    RAWRTC_SCTP_TRANSPORT_CHECKSUM_DISABLE_ALL = (1 << 0 | 1 << 1),
};

/**
 * SCTP transport state.
 */
enum rawrtc_sctp_transport_state {
    RAWRTC_SCTP_TRANSPORT_STATE_NEW,
    RAWRTC_SCTP_TRANSPORT_STATE_CONNECTING,
    RAWRTC_SCTP_TRANSPORT_STATE_CONNECTED,
    RAWRTC_SCTP_TRANSPORT_STATE_CLOSED,
};

/**
 * SCTP transport congestion control algorithm.
 *
 * Note: RFC 2581 is the original default algorithm for TCP.
 */
enum rawrtc_sctp_transport_congestion_ctrl {
    RAWRTC_SCTP_TRANSPORT_CONGESTION_CTRL_RFC2581,
    RAWRTC_SCTP_TRANSPORT_CONGESTION_CTRL_HSTCP,
    RAWRTC_SCTP_TRANSPORT_CONGESTION_CTRL_HTCP,
    RAWRTC_SCTP_TRANSPORT_CONGESTION_CTRL_RTCC,
};

/**
 * SCTP transport default MTU.
 */
enum {
    RAWRTC_SCTP_TRANSPORT_DEFAULT_MTU = 1200,
};

/**
 * SCTP transport.
 */
struct rawrtc_sctp_transport;

/**
 * SCTP transport state change handler.
 */
typedef void (*rawrtc_sctp_transport_state_change_handler)(
    enum rawrtc_sctp_transport_state const state, void* const arg);

/**
 * Create an SCTP transport from an external DTLS transport.
 * `*transportp` must be unreferenced.
 *
 * Note: The underlying DTLS transport is supposed to be immediately
 *       attached after creation of this transport.
 */
enum rawrtc_code rawrtc_sctp_transport_create_from_external(
    struct rawrtc_sctp_transport** const transportp,  // de-referenced
    struct rawrtc_sctp_transport_context* const context,  // copied
    uint16_t port,  // zeroable
    rawrtc_data_channel_handler const data_channel_handler,  // nullable
    rawrtc_sctp_transport_state_change_handler const state_change_handler,  // nullable
    void* const arg  // nullable
);

/**
 * Get the SCTP data transport instance.
 * `*transportp` must be unreferenced.
 */
enum rawrtc_code rawrtc_sctp_transport_get_data_transport(
    struct rawrtc_data_transport** const transportp,  // de-referenced
    struct rawrtc_sctp_transport* const sctp_transport  // referenced
);

/**
 * Start the SCTP transport.
 */
enum rawrtc_code rawrtc_sctp_transport_start(
    struct rawrtc_sctp_transport* const transport,
    struct rawrtc_sctp_capabilities const* const remote_capabilities,  // copied
    uint16_t remote_port  // zeroable
);

/**
 * Stop and close the SCTP transport.
 */
enum rawrtc_code rawrtc_sctp_transport_stop(struct rawrtc_sctp_transport* const transport);

/**
 * Feed inbound data to the SCTP transport.
 *
 * `buffer` contains the data to be fed to the SCTP transport. Since
 * the data is not going to be referenced, you can pass a *fake* `mbuf`
 * structure that hasn't been allocated with `mbuf_alloc` to avoid
 * copying.
 * `ecn_bits` are the explicit congestion notification bits to be
 * passed to usrsctp.
 *
 * Return `RAWRTC_CODE_INVALID_STATE` in case the transport is closed.
 * Otherwise, `RAWRTC_CODE_SUCCESS` is being returned.
 */
enum rawrtc_code rawrtc_sctp_transport_feed_inbound(
    struct rawrtc_sctp_transport* const transport,
    struct mbuf* const buffer,
    uint8_t const ecn_bits);

/**
 * Set the SCTP transport's send and receive buffer length in bytes.
 */
enum rawrtc_code rawrtc_sctp_transport_set_buffer_length(
    struct rawrtc_sctp_transport* const transport,
    uint32_t const send_buffer_length,
    uint32_t const receive_buffer_length);

/**
 * Get the SCTP transport's send and receive buffer length in bytes.
 */
enum rawrtc_code rawrtc_sctp_transport_get_buffer_length(
    uint32_t* const send_buffer_lengthp,
    uint32_t* const receive_buffer_lengthp,
    struct rawrtc_sctp_transport* const transport);

/**
 * Set the SCTP transport's congestion control algorithm.
 */
enum rawrtc_code rawrtc_sctp_transport_set_congestion_ctrl_algorithm(
    struct rawrtc_sctp_transport* const transport,
    enum rawrtc_sctp_transport_congestion_ctrl const algorithm);

/**
 * Get the current SCTP transport's congestion control algorithm.
 */
enum rawrtc_code rawrtc_sctp_transport_get_congestion_ctrl_algorithm(
    enum rawrtc_sctp_transport_congestion_ctrl* const algorithmp,
    struct rawrtc_sctp_transport* const transport);

/**
 * Set the SCTP transport's maximum transmission unit (MTU).
 * This will disable MTU discovery.
 *
 * Note: The MTU cannot be set before the SCTP transport has been
 *       started.
 */
enum rawrtc_code rawrtc_sctp_transport_set_mtu(
    struct rawrtc_sctp_transport* const transport, uint32_t mtu);

/**
 * Get the current SCTP transport's maximum transmission unit (MTU)
 * and an indication whether MTU discovery is enabled.
 *
 * Note: The MTU cannot be retrieved before the SCTP transport has been
 *       started.
 */
enum rawrtc_code rawrtc_sctp_transport_get_mtu(
    uint32_t* const mtup,  // de-referenced
    bool* const mtu_discovery_enabledp,  // de-referenced
    struct rawrtc_sctp_transport* const transport);

/**
 * Enable MTU discovery for the SCTP transport.
 *
 * Note: MTU discovery cannot be enabled before the SCTP transport has
 *       been started.
 */
enum rawrtc_code rawrtc_sctp_transport_enable_mtu_discovery(
    struct rawrtc_sctp_transport* const transport);

/**
 * Set the SCTP transport's context.
 */
enum rawrtc_code rawrtc_sctp_transport_set_context(
    struct rawrtc_sctp_transport* const transport,
    struct rawrtc_sctp_transport_context* const context  // copied
);

/**
 * Get the current state of the SCTP transport.
 */
enum rawrtc_code rawrtc_sctp_transport_get_state(
    enum rawrtc_sctp_transport_state* const statep,  // de-referenced
    struct rawrtc_sctp_transport* const transport);

/**
 * Get the local port of the SCTP transport.
 */
enum rawrtc_code rawrtc_sctp_transport_get_port(
    uint16_t* const portp,  // de-referenced
    struct rawrtc_sctp_transport* const transport);

/**
 * Get the number of streams allocated for the SCTP transport.
 */
enum rawrtc_code rawrtc_sctp_transport_get_n_streams(
    uint16_t* const n_streamsp,  // de-referenced
    struct rawrtc_sctp_transport* const transport);

/**
 * Get the local SCTP transport capabilities (static).
 * `*capabilitiesp` must be unreferenced.
 */
enum rawrtc_code rawrtc_sctp_transport_get_capabilities(
    struct rawrtc_sctp_capabilities** const capabilitiesp  // de-referenced
);

/**
 * Set the SCTP transport's data channel handler.
 */
enum rawrtc_code rawrtc_sctp_transport_set_data_channel_handler(
    struct rawrtc_sctp_transport* const transport,
    rawrtc_data_channel_handler const data_channel_handler  // nullable
);

/**
 * Get the SCTP transport's data channel handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_sctp_transport_get_data_channel_handler(
    rawrtc_data_channel_handler* const data_channel_handlerp,  // de-referenced
    struct rawrtc_sctp_transport* const transport);

/**
 * Set the SCTP transport's state change handler.
 */
enum rawrtc_code rawrtc_sctp_transport_set_state_change_handler(
    struct rawrtc_sctp_transport* const transport,
    rawrtc_sctp_transport_state_change_handler const state_change_handler  // nullable
);

/**
 * Get the SCTP transport's state change handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_sctp_transport_get_state_change_handler(
    rawrtc_sctp_transport_state_change_handler* const state_change_handlerp,  // de-referenced
    struct rawrtc_sctp_transport* const transport);

/**
 * Get the corresponding name for an SCTP transport state.
 */
char const* rawrtc_sctp_transport_state_to_name(enum rawrtc_sctp_transport_state const state);

/*
 * Get the corresponding name for a congestion control algorithm.
 */
char const* rawrtc_sctp_transport_congestion_ctrl_algorithm_to_name(
    enum rawrtc_sctp_transport_congestion_ctrl const algorithm);
