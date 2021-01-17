#pragma once
#include <rawrtcc/code.h>
#include <re.h>

/**
 * External DTLS role.
 */
enum rawrtc_external_dtls_role {
    RAWRTC_EXTERNAL_DTLS_ROLE_CLIENT,
    RAWRTC_EXTERNAL_DTLS_ROLE_SERVER,
};

/**
 * External DTLS transport state.
 */
enum rawrtc_external_dtls_transport_state {
    RAWRTC_EXTERNAL_DTLS_TRANSPORT_STATE_NEW_OR_CONNECTING,
    RAWRTC_EXTERNAL_DTLS_TRANSPORT_STATE_CONNECTED,
    RAWRTC_EXTERNAL_DTLS_TRANSPORT_STATE_CLOSED_OR_FAILED,
};

/**
 * DTLS role getter.
 *
 * `*rolep` will contain the current external DTLS role.
 * `arg` is the argument passed to the SCTP transport context.
 *
 * Return `RAWRTC_CODE_SUCCESS` in case the role has been set or any
 * other code in case of an error.
 */
typedef enum rawrtc_code (*rawrtc_dtls_role_getter)(
    enum rawrtc_external_dtls_role* const rolep,  // de-referenced
    void* const arg);

/**
 * DTLS transport state getter.
 *
 * `*statep` will contain the current external DTLS transport state.
 * `arg` is the argument passed to the SCTP transport context.
 *
 * Return `RAWRTC_CODE_SUCCESS` in case the state has been set or any
 * other code in case of an error.
 */
typedef enum rawrtc_code (*rawrtc_dtls_transport_state_getter)(
    enum rawrtc_external_dtls_transport_state* const statep,  // de-referenced
    void* const arg);

/**
 * SCTP transport outbound data handler.
 *
 * `buffer` contains the data to be fed to the DTLS transport.
 * Note that the `mbuf` structure shall not be `mem_ref`ed or
 * `mem_deref`ed since it hasn't been allocated properly for
 * optimisation purposes. This has been done since we expect you to
 * either send this data directly or drop it. There's no need to hold
 * data back. If you for any reason need the data after the callback
 * returned, you are required to copy it.
 * `tos` contains the type of service field as reported by usrsctp.
 * `set_df` TODO: Probably don't fragment bit? Dunno...
 *
 * Return `RAWRTC_CODE_SUCCESS` in case the packet has been sent (or
 * dropped) or any other code in case of an error.
 */
typedef enum rawrtc_code (*rawrtc_sctp_transport_outbound_handler)(
    struct mbuf* const buffer, uint8_t const tos, uint8_t const set_df, void* const arg);

/**
 * SCTP transport detach handler.
 * Will be called when the SCTP transport is about to be closed and
 * should be detached from the underlying DTLS transport. At this
 * point, no further data should be fed to the SCTP transport.
 *
 * `arg` is the argument passed to the SCTP transport context.
 */
typedef void (*rawrtc_sctp_transport_detach_handler)(void* const arg);

/**
 * SCTP transport destroyed handler.
 * Will be called when the SCTP transport is about to be free'd.
 *
 * Note: This handler only exists for cleanup purposes. You may not use
 *       any of the transport's functions at this point.
 *
 * `arg` is the argument passed to the SCTP transport context.
 */
typedef void (*rawrtc_sctp_transport_destroyed_handler)(void* const arg);

/**
 * SCTP transport context.
 */
struct rawrtc_sctp_transport_context {
    rawrtc_dtls_role_getter role_getter;
    rawrtc_dtls_transport_state_getter state_getter;
    rawrtc_sctp_transport_outbound_handler outbound_handler;
    rawrtc_sctp_transport_detach_handler detach_handler;  // nullable
    rawrtc_sctp_transport_destroyed_handler destroyed_handler;  // nullable
    bool trace_packets;
    void* arg;  // nullable
};
