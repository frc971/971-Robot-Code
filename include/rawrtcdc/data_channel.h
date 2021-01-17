#pragma once
#include <rawrtcc/code.h>
#include <re.h>

// Dependencies
struct rawrtc_data_channel_parameters;
struct rawrtc_data_transport;

/**
 * Data channel types.
 */
enum rawrtc_data_channel_type {
    RAWRTC_DATA_CHANNEL_TYPE_RELIABLE_ORDERED = 0x00,
    RAWRTC_DATA_CHANNEL_TYPE_RELIABLE_UNORDERED = 0x80,
    RAWRTC_DATA_CHANNEL_TYPE_UNRELIABLE_ORDERED_RETRANSMIT = 0x01,
    RAWRTC_DATA_CHANNEL_TYPE_UNRELIABLE_UNORDERED_RETRANSMIT = 0x81,
    RAWRTC_DATA_CHANNEL_TYPE_UNRELIABLE_ORDERED_TIMED = 0x02,
    RAWRTC_DATA_CHANNEL_TYPE_UNRELIABLE_UNORDERED_TIMED = 0x82,
};  // IMPORTANT: If you add a new type, ensure that every data channel transport handles it
    //            correctly! Also, ensure this still works with the unordered bit flag above or
    //            update the implementations.

/**
 * Data channel message flags.
 */
enum rawrtc_data_channel_message_flag {
    RAWRTC_DATA_CHANNEL_MESSAGE_FLAG_IS_ABORTED = 1 << 0,
    RAWRTC_DATA_CHANNEL_MESSAGE_FLAG_IS_COMPLETE = 1 << 1,
    RAWRTC_DATA_CHANNEL_MESSAGE_FLAG_IS_STRING = 1 << 2,
    RAWRTC_DATA_CHANNEL_MESSAGE_FLAG_IS_BINARY = 1 << 3,
};

/**
 * Data channel state.
 */
enum rawrtc_data_channel_state {
    RAWRTC_DATA_CHANNEL_STATE_CONNECTING,
    RAWRTC_DATA_CHANNEL_STATE_OPEN,
    RAWRTC_DATA_CHANNEL_STATE_CLOSING,
    RAWRTC_DATA_CHANNEL_STATE_CLOSED,
};

/**
 * Data channel.
 */
struct rawrtc_data_channel;

/**
 * Data channel open handler.
 */
typedef void (*rawrtc_data_channel_open_handler)(void* const arg);

/**
 * Data channel buffered amount low handler.
 */
typedef void (*rawrtc_data_channel_buffered_amount_low_handler)(void* const arg);

/**
 * Data channel error handler.
 */
typedef void (*rawrtc_data_channel_error_handler)(void* const arg);  // TODO: error parameter

/**
 * Data channel close handler.
 */
typedef void (*rawrtc_data_channel_close_handler)(void* const arg);

/**
 * Data channel message handler.
 *
 * Note: `buffer` may be NULL in case partial delivery has been
 *       requested and a message has been aborted (this can only happen
 *       on partially reliable channels).
 */
typedef void (*rawrtc_data_channel_message_handler)(
    struct mbuf* const buffer,  // nullable (in case partial delivery has been requested)
    enum rawrtc_data_channel_message_flag const flags,
    void* const arg);

/**
 * Data channel handler.
 *
 * Note: You should call `rawrtc_data_channel_set_streaming`
 *       in this handler before doing anything else if you want to
 *       enable streamed delivery of data for this channel from the
 *       beginning of the first incoming message.
 */
typedef void (*rawrtc_data_channel_handler)(
    struct rawrtc_data_channel* const data_channel,  // read-only, MUST be referenced when used
    void* const arg);

/**
 * Create a data channel.
 * `*channelp` must be unreferenced.
 *
 * Note: You should call `rawrtc_data_channel_set_streaming`
 *       directly after this function returned if you want to enable
 *       streamed delivery of data for this channel from the beginning
 *       of the first incoming message.
 */
enum rawrtc_code rawrtc_data_channel_create(
    struct rawrtc_data_channel** const channelp,  // de-referenced
    struct rawrtc_data_transport* const transport,  // referenced
    struct rawrtc_data_channel_parameters* const parameters,  // referenced
    rawrtc_data_channel_open_handler const open_handler,  // nullable
    rawrtc_data_channel_buffered_amount_low_handler const buffered_amount_low_handler,  // nullable
    rawrtc_data_channel_error_handler const error_handler,  // nullable
    rawrtc_data_channel_close_handler const close_handler,  // nullable
    rawrtc_data_channel_message_handler const message_handler,  // nullable
    void* const arg  // nullable
);

/**
 * Set the argument of a data channel that is passed to the various
 * handlers.
 */
enum rawrtc_code rawrtc_data_channel_set_arg(
    struct rawrtc_data_channel* const channel,
    void* const arg  // nullable
);

/**
 * Send data via the data channel.
 */
enum rawrtc_code rawrtc_data_channel_send(
    struct rawrtc_data_channel* const channel,
    struct mbuf* const buffer,  // nullable (if empty message), referenced
    bool const is_binary);

/**
 * Close the data channel.
 */
enum rawrtc_code rawrtc_data_channel_close(struct rawrtc_data_channel* const channel);

/**
 * Get the current state of the data channel.
 */
enum rawrtc_code rawrtc_data_channel_get_state(
    enum rawrtc_data_channel_state* const statep,  // de-referenced
    struct rawrtc_data_channel* const channel);

/**
 * Get the currently buffered amount (bytes) of outgoing application
 * data of the data channel.
 */
enum rawrtc_code rawrtc_data_channel_get_buffered_amount(
    uint64_t* const buffered_amountp,  // de-referenced
    struct rawrtc_data_channel* const channel);

/**
 * Set the data channel's buffered amount (bytes) low threshold for
 * outgoing application data.
 */
enum rawrtc_code rawrtc_data_channel_set_buffered_amount_low_threshold(
    struct rawrtc_data_channel* const channel, uint64_t const buffered_amount_low_threshold);

/**
 * Get the data channel's buffered amount (bytes) low threshold for
 * outgoing application data.
 */
enum rawrtc_code rawrtc_data_channel_get_buffered_amount_low_threshold(
    uint64_t* const buffered_amount_low_thresholdp,  // de-referenced
    struct rawrtc_data_channel* const channel);

/**
 * Unset the handler argument and all handlers of the data channel.
 */
enum rawrtc_code rawrtc_data_channel_unset_handlers(struct rawrtc_data_channel* const channel);

/**
 * Get the data channel's parameters.
 * `*parametersp` must be unreferenced.
 */
enum rawrtc_code rawrtc_data_channel_get_parameters(
    struct rawrtc_data_channel_parameters** const parametersp,  // de-referenced
    struct rawrtc_data_channel* const channel);

/**
 * Enable or disable streamed delivery.
 *
 * Note: In case an incoming message is currently pending (there are
 *       queued chunks in the internal reassembly buffer), this will
 *       fail with an *invalid state* error.
 */
enum rawrtc_code rawrtc_data_channel_set_streaming(
    struct rawrtc_data_channel* const channel, bool const on);

/**
 * Set the data channel's open handler.
 */
enum rawrtc_code rawrtc_data_channel_set_open_handler(
    struct rawrtc_data_channel* const channel,
    rawrtc_data_channel_open_handler const open_handler  // nullable
);

/**
 * Get the data channel's open handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_data_channel_get_open_handler(
    rawrtc_data_channel_open_handler* const open_handlerp,  // de-referenced
    struct rawrtc_data_channel* const channel);

/**
 * Set the data channel's buffered amount low handler.
 */
enum rawrtc_code rawrtc_data_channel_set_buffered_amount_low_handler(
    struct rawrtc_data_channel* const channel,
    rawrtc_data_channel_buffered_amount_low_handler const buffered_amount_low_handler  // nullable
);

/**
 * Get the data channel's buffered amount low handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_data_channel_get_buffered_amount_low_handler(
    rawrtc_data_channel_buffered_amount_low_handler* const
        buffered_amount_low_handlerp,  // de-referenced
    struct rawrtc_data_channel* const channel);

/**
 * Set the data channel's error handler.
 */
enum rawrtc_code rawrtc_data_channel_set_error_handler(
    struct rawrtc_data_channel* const channel,
    rawrtc_data_channel_error_handler const error_handler  // nullable
);

/**
 * Get the data channel's error handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_data_channel_get_error_handler(
    rawrtc_data_channel_error_handler* const error_handlerp,  // de-referenced
    struct rawrtc_data_channel* const channel);

/**
 * Set the data channel's close handler.
 */
enum rawrtc_code rawrtc_data_channel_set_close_handler(
    struct rawrtc_data_channel* const channel,
    rawrtc_data_channel_close_handler const close_handler  // nullable
);

/**
 * Get the data channel's close handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_data_channel_get_close_handler(
    rawrtc_data_channel_close_handler* const close_handlerp,  // de-referenced
    struct rawrtc_data_channel* const channel);

/**
 * Set the data channel's message handler.
 */
enum rawrtc_code rawrtc_data_channel_set_message_handler(
    struct rawrtc_data_channel* const channel,
    rawrtc_data_channel_message_handler const message_handler  // nullable
);

/**
 * Get the data channel's message handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_data_channel_get_message_handler(
    rawrtc_data_channel_message_handler* const message_handlerp,  // de-referenced
    struct rawrtc_data_channel* const channel);

/**
 * Get the corresponding name for a data channel state.
 */
char const* rawrtc_data_channel_state_to_name(enum rawrtc_data_channel_state const state);
