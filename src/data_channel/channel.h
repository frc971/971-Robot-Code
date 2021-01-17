#pragma once
#include <rawrtcdc/data_channel.h>
#include <rawrtcdc/data_channel_parameters.h>
#include <rawrtcdc/data_transport.h>
#include <rawrtcc/code.h>
#include <re.h>

/*
 * Data channel flags.
 */
enum {
    RAWRTC_DATA_CHANNEL_FLAGS_INITIALIZED = 1 << 0,
    RAWRTC_DATA_CHANNEL_FLAGS_STREAMED = 1 << 1,
};

/*
 * Data channel type unordered bit flag.
 */
enum {
    RAWRTC_DATA_CHANNEL_TYPE_IS_UNORDERED = 0x80,
};

/*
 * Data channel.
 */
struct rawrtc_data_channel {
    uint_fast8_t flags;
    enum rawrtc_data_channel_state state;
    struct rawrtc_data_transport* transport;  // referenced
    void* transport_arg;  // referenced
    struct rawrtc_data_channel_parameters* parameters;  // referenced
    rawrtc_data_channel_open_handler open_handler;  // nullable
    rawrtc_data_channel_buffered_amount_low_handler buffered_amount_low_handler;  // nullable
    rawrtc_data_channel_error_handler error_handler;  // nullable
    rawrtc_data_channel_close_handler close_handler;  // nullable
    rawrtc_data_channel_message_handler message_handler;  // nullable
    void* arg;  // nullable
};

void rawrtc_data_channel_set_state(
    struct rawrtc_data_channel* const channel, enum rawrtc_data_channel_state const state);

enum rawrtc_code rawrtc_data_channel_create_internal(
    struct rawrtc_data_channel** const channelp,  // de-referenced
    struct rawrtc_data_transport* const transport,  // referenced
    struct rawrtc_data_channel_parameters* const parameters,  // referenced
    rawrtc_data_channel_open_handler const open_handler,  // nullable
    rawrtc_data_channel_buffered_amount_low_handler const buffered_amount_low_handler,  // nullable
    rawrtc_data_channel_error_handler const error_handler,  // nullable
    rawrtc_data_channel_close_handler const close_handler,  // nullable
    rawrtc_data_channel_message_handler const message_handler,  // nullable
    void* const arg,  // nullable
    bool const call_handler);

void rawrtc_data_channel_call_channel_handler(
    struct rawrtc_data_channel* const channel,  // not checked
    rawrtc_data_channel_handler const channel_handler,  // nullable
    void* const arg);
