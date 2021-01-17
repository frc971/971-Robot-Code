#pragma once
#include <rawrtcdc/data_channel.h>
#include <rawrtcdc/external.h>
#include <rawrtcdc/sctp_transport.h>
#include <re.h>
#include <usrsctp.h>  // SCTP_EVENT_*, ...
#include <stdio.h>  // FILE

/*
 * usrsctp event flag extensions for handlers.
 */
#define RAWRTC_SCTP_EVENT_NONE (0)
#define RAWRTC_SCTP_EVENT_ALL (SCTP_EVENT_READ | SCTP_EVENT_WRITE | SCTP_EVENT_ERROR)

enum {
    RAWRTC_SCTP_TRANSPORT_MAX_MESSAGE_SIZE = 0,
    RAWRTC_SCTP_TRANSPORT_TIMER_TIMEOUT = 10,
    RAWRTC_SCTP_TRANSPORT_DEFAULT_PORT = 5000,
    // As specified by https://tools.ietf.org/html/draft-ietf-rtcweb-data-channel-13#section-5
    RAWRTC_SCTP_TRANSPORT_DEFAULT_NUMBER_OF_STREAMS = 65535,
    RAWRTC_SCTP_TRANSPORT_SID_MAX = 65534,
    RAWRTC_SCTP_TRANSPORT_EMPTY_MESSAGE_SIZE = 1,
};

/*
 * SCTP transport flags.
 */
enum {
    RAWRTC_SCTP_TRANSPORT_FLAGS_INITIALIZED = 1 << 0,
    // The detached flag is virtually identical to the 'closed' state but is applied before the
    // detach handler is being called. Thus, any other functions should check for the detached flag
    // instead of checking for the 'closed' state since that is being set at a later stage.
    RAWRTC_SCTP_TRANSPORT_FLAGS_DETACHED = 1 << 1,
    RAWRTC_SCTP_TRANSPORT_FLAGS_SENDING_IN_PROGRESS = 1 << 2,
    RAWRTC_SCTP_TRANSPORT_FLAGS_BUFFERED_AMOUNT_LOW = 1 << 3,
};

/*
 * SCTP data channel flags.
 */
enum {
    RAWRTC_SCTP_DATA_CHANNEL_FLAGS_CAN_SEND_UNORDERED = 1 << 0,
    RAWRTC_SCTP_DATA_CHANNEL_FLAGS_PENDING_INBOUND_MESSAGE = 1 << 1,
    RAWRTC_SCTP_DATA_CHANNEL_FLAGS_PENDING_STREAM_RESET = 1 << 2,
    RAWRTC_SCTP_DATA_CHANNEL_FLAGS_INCOMING_STREAM_RESET = 1 << 3,
    RAWRTC_SCTP_DATA_CHANNEL_FLAGS_OUTGOING_STREAM_RESET = 1 << 4,
};

/*
 * DCEP message types.
 */
enum {
    RAWRTC_DCEP_MESSAGE_TYPE_ACK = 0x02,
    RAWRTC_DCEP_MESSAGE_TYPE_OPEN = 0x03,
};

/*
 * DCEP message sizes
 */
enum {
    RAWRTC_DCEP_MESSAGE_ACK_BASE_SIZE = 1,
    RAWRTC_DCEP_MESSAGE_OPEN_BASE_SIZE = 12,
};

/*
 * DCEP message priorities.
 */
enum {
    RAWRTC_DCEP_CHANNEL_PRIORITY_LOW = 128,
    RAWRTC_DCEP_CHANNEL_PRIORITY_NORMAL = 256,
    RAWRTC_DCEP_CHANNEL_PRIORITY_HIGH = 512,
    RAWRTC_DCEP_CHANNEL_PRIORITY_EXTRA_HIGH = 1024,
};

/*
 * DCEP payload protocol identifiers.
 */
enum {
    RAWRTC_SCTP_TRANSPORT_PPID_DCEP = 50,
    RAWRTC_SCTP_TRANSPORT_PPID_UTF16 = 51,
    RAWRTC_SCTP_TRANSPORT_PPID_UTF16_EMPTY = 56,
    RAWRTC_SCTP_TRANSPORT_PPID_UTF16_PARTIAL = 54,  // deprecated
    RAWRTC_SCTP_TRANSPORT_PPID_BINARY = 53,
    RAWRTC_SCTP_TRANSPORT_PPID_BINARY_EMPTY = 57,
    RAWRTC_SCTP_TRANSPORT_PPID_BINARY_PARTIAL = 52,  // deprecated
};

/*
 * SCTP transport.
 */
struct rawrtc_sctp_transport {
    struct rawrtc_sctp_transport_context context;
    uint_fast8_t flags;
    enum rawrtc_sctp_transport_state state;
    uint16_t port;
    struct sockaddr_conn remote_address;
    uint64_t remote_maximum_message_size;
    rawrtc_data_channel_handler data_channel_handler;  // nullable
    rawrtc_sctp_transport_state_change_handler state_change_handler;  // nullable
    void* arg;  // nullable
    struct list buffered_messages_outgoing;
    struct mbuf* buffer_dcep_inbound;
    struct sctp_rcvinfo info_dcep_inbound;
    struct rawrtc_data_channel** channels;
    uint_fast16_t n_channels;
    uint_fast16_t current_channel_sid;
    FILE* trace_handle;
    struct socket* socket;
};

/*
 * Contextual data required by the SCTP transport used in conjunction with a
 * data channel.
 */
struct rawrtc_sctp_data_channel_context {
    uint16_t sid;
    uint_fast8_t flags;
    struct mbuf* buffer_inbound;
    struct sctp_rcvinfo info_inbound;
};
