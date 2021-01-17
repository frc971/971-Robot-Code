#pragma once
#include <rawrtcdc/external.h>
#include <rawrtcdc/sctp_redirect_transport.h>
#include <rawrtcdc/sctp_transport.h>
#include <re.h>

enum {
    RAWRTC_SCTP_REDIRECT_TRANSPORT_DEFAULT_PORT = 5000,
    RAWRTC_SCTP_REDIRECT_TRANSPORT_RAW_SOCKET_RECEIVE_SIZE = 8192,
};

/*
 * SCTP redirect transport flags.
 */
enum {
    RAWRTC_SCTP_REDIRECT_TRANSPORT_FLAGS_INITIALIZED = 1 << 0,
    // The detached flag is virtually identical to the 'closed' state but is applied before the
    // detach handler is being called. Thus, any other functions should check for the detached flag
    // instead of checking for the 'closed' state since that is being set at a later stage.
    RAWRTC_SCTP_REDIRECT_TRANSPORT_FLAGS_DETACHED = 1 << 1,
};

/*
 * SCTP redirect transport.
 */
struct rawrtc_sctp_redirect_transport {
    struct rawrtc_sctp_transport_context context;
    uint_fast8_t flags;
    enum rawrtc_sctp_redirect_transport_state state;
    uint16_t local_port;
    uint16_t remote_port;
    struct sa redirect_address;
    rawrtc_sctp_redirect_transport_state_change_handler state_change_handler;  // nullable
    void* arg;  // nullable
    struct mbuf* buffer;
    int socket;
};
