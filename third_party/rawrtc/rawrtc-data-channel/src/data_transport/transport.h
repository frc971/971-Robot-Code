#pragma once
#include <rawrtcdc/data_channel.h>
#include <rawrtcdc/data_channel_parameters.h>
#include <rawrtcdc/data_transport.h>
#include <rawrtcc/code.h>
#include <re.h>

/*
 * Create the data channel (transport handler).
 */
typedef enum rawrtc_code (*rawrtc_data_transport_channel_create_handler)(
    struct rawrtc_data_transport* const transport,
    struct rawrtc_data_channel* const channel,  // referenced
    struct rawrtc_data_channel_parameters const* const parameters  // read-only
);

/*
 * Close the data channel (transport handler).
 */
typedef enum rawrtc_code (*rawrtc_data_transport_channel_close_handler)(
    struct rawrtc_data_channel* const channel);

/*
 * Send data via the data channel (transport handler).
 */
typedef enum rawrtc_code (*rawrtc_data_transport_channel_send_handler)(
    struct rawrtc_data_channel* const channel,
    struct mbuf* buffer,  // nullable (if size 0), referenced
    bool const is_binary);

/*
 * Check if the data transport allows changing the delivery mode
 * (transport handler).
 */
typedef enum rawrtc_code (*rawrtc_data_transport_channel_set_streaming_handler)(
    struct rawrtc_data_channel* const channel, bool const on);

/**
 * Generic data transport.
 */
struct rawrtc_data_transport {
    enum rawrtc_data_transport_type type;
    void* transport;
    rawrtc_data_transport_channel_create_handler channel_create;
    rawrtc_data_transport_channel_close_handler channel_close;
    rawrtc_data_transport_channel_send_handler channel_send;
    rawrtc_data_transport_channel_set_streaming_handler channel_set_streaming;
};

enum rawrtc_code rawrtc_data_transport_create(
    struct rawrtc_data_transport** const transportp,  // de-referenced
    enum rawrtc_data_transport_type const type,
    void* const internal_transport,  // referenced
    rawrtc_data_transport_channel_create_handler const channel_create_handler,
    rawrtc_data_transport_channel_close_handler const channel_close_handler,
    rawrtc_data_transport_channel_send_handler const channel_send_handler,
    rawrtc_data_transport_channel_set_streaming_handler const channel_set_streaming_handler);
