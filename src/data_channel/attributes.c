#include "channel.h"
#include "../data_transport/transport.h"
#include <rawrtcdc/config.h>
#include <rawrtcdc/data_channel.h>
#include <rawrtcdc/data_channel_parameters.h>
#include <rawrtcc/code.h>
#include <re.h>

#define DEBUG_MODULE "data-channel"
//#define RAWRTC_DEBUG_MODULE_LEVEL 7 // Note: Uncomment this to debug this module only
#include <rawrtcc/debug.h>

/*
 * Get the current state of the data channel.
 */
enum rawrtc_code rawrtc_data_channel_get_state(
    enum rawrtc_data_channel_state* const statep,  // de-referenced
    struct rawrtc_data_channel* const channel) {
    // Check arguments
    if (!statep || !channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set state & done
    *statep = channel->state;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the currently buffered amount (bytes) of outgoing application
 * data of the data channel.
 */
enum rawrtc_code rawrtc_data_channel_get_buffered_amount(
    uint64_t* const buffered_amountp,  // de-referenced
    struct rawrtc_data_channel* const channel) {
    // Check arguments
    if (!buffered_amountp || !channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // TODO: Implement this!
    return RAWRTC_CODE_NOT_IMPLEMENTED;
}

/*
 * Set the data channel's buffered amount (bytes) low threshold for
 * outgoing application data.
 */
enum rawrtc_code rawrtc_data_channel_set_buffered_amount_low_threshold(
    struct rawrtc_data_channel* const channel, uint64_t const buffered_amount_low_threshold) {
    // Check arguments
    if (!channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // TODO: Implement this!
    (void) buffered_amount_low_threshold;
    return RAWRTC_CODE_NOT_IMPLEMENTED;
}

/*
 * Get the data channel's buffered amount (bytes) low threshold for
 * outgoing application data.
 */
enum rawrtc_code rawrtc_data_channel_get_buffered_amount_low_threshold(
    uint64_t* const buffered_amount_low_thresholdp,  // de-referenced
    struct rawrtc_data_channel* const channel) {
    // Check arguments
    if (!buffered_amount_low_thresholdp || !channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // TODO: Implement this!
    return RAWRTC_CODE_NOT_IMPLEMENTED;
}

/*
 * Unset the handler argument and all handlers of the data channel.
 */
enum rawrtc_code rawrtc_data_channel_unset_handlers(struct rawrtc_data_channel* const channel) {
    // Check arguments
    if (!channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Unset handler argument
    channel->arg = NULL;

    // Unset all handlers
    channel->message_handler = NULL;
    channel->close_handler = NULL;
    channel->error_handler = NULL;
    channel->buffered_amount_low_handler = NULL;
    channel->open_handler = NULL;

    // Done
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the data channel's parameters.
 * `*parametersp` must be unreferenced.
 */
enum rawrtc_code rawrtc_data_channel_get_parameters(
    struct rawrtc_data_channel_parameters** const parametersp,  // de-referenced
    struct rawrtc_data_channel* const channel) {
    // Check arguments
    if (!parametersp || !channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set pointer & done
    *parametersp = mem_ref(channel->parameters);
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Enable or disable streamed delivery.
 *
 * Note: In case an incoming message is currently pending (there are
 *       queued chunks in the internal reassembly buffer), this will
 *       fail with a *still in use* error.
 */
enum rawrtc_code rawrtc_data_channel_set_streaming(
    struct rawrtc_data_channel* const channel, bool const on) {
    enum rawrtc_code error;

    // Check arguments
    if (!channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Check state
    if (channel->state == RAWRTC_DATA_CHANNEL_STATE_CLOSING ||
        channel->state == RAWRTC_DATA_CHANNEL_STATE_CLOSED) {
        return RAWRTC_CODE_INVALID_STATE;
    }

    // Does anything change?
    if ((on && channel->flags & RAWRTC_DATA_CHANNEL_FLAGS_STREAMED) ||
        (!on && !(channel->flags & RAWRTC_DATA_CHANNEL_FLAGS_STREAMED))) {
        return RAWRTC_CODE_SUCCESS;
    }

    // Let the transport know we want to enable/disable streaming
    error = channel->transport->channel_set_streaming(channel, on);
    if (error) {
        return error;
    }

    // Enable/disable streaming & done
    if (on) {
        channel->flags |= RAWRTC_DATA_CHANNEL_FLAGS_STREAMED;
        DEBUG_PRINTF("Enabled streaming mode\n");
    } else {
        channel->flags &= ~RAWRTC_DATA_CHANNEL_FLAGS_STREAMED;
        DEBUG_PRINTF("Disabled streaming mode\n");
    }
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Set the data channel's open handler.
 */
enum rawrtc_code rawrtc_data_channel_set_open_handler(
    struct rawrtc_data_channel* const channel,
    rawrtc_data_channel_open_handler const open_handler  // nullable
) {
    // Check arguments
    if (!channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set open handler & done
    channel->open_handler = open_handler;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the data channel's open handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_data_channel_get_open_handler(
    rawrtc_data_channel_open_handler* const open_handlerp,  // de-referenced
    struct rawrtc_data_channel* const channel) {
    // Check arguments
    if (!open_handlerp || !channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Get open handler (if any)
    if (channel->open_handler) {
        *open_handlerp = channel->open_handler;
        return RAWRTC_CODE_SUCCESS;
    } else {
        return RAWRTC_CODE_NO_VALUE;
    }
}

/*
 * Set the data channel's buffered amount low handler.
 */
enum rawrtc_code rawrtc_data_channel_set_buffered_amount_low_handler(
    struct rawrtc_data_channel* const channel,
    rawrtc_data_channel_buffered_amount_low_handler const buffered_amount_low_handler  // nullable
) {
    // Check arguments
    if (!channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set buffered amount low handler & done
    channel->buffered_amount_low_handler = buffered_amount_low_handler;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the data channel's buffered amount low handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_data_channel_get_buffered_amount_low_handler(
    rawrtc_data_channel_buffered_amount_low_handler* const
        buffered_amount_low_handlerp,  // de-referenced
    struct rawrtc_data_channel* const channel) {
    // Check arguments
    if (!buffered_amount_low_handlerp || !channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Get buffered amount low handler (if any)
    if (channel->buffered_amount_low_handler) {
        *buffered_amount_low_handlerp = channel->buffered_amount_low_handler;
        return RAWRTC_CODE_SUCCESS;
    } else {
        return RAWRTC_CODE_NO_VALUE;
    }
}

/*
 * Set the data channel's error handler.
 */
enum rawrtc_code rawrtc_data_channel_set_error_handler(
    struct rawrtc_data_channel* const channel,
    rawrtc_data_channel_error_handler const error_handler  // nullable
) {
    // Check arguments
    if (!channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set error handler & done
    channel->error_handler = error_handler;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the data channel's error handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_data_channel_get_error_handler(
    rawrtc_data_channel_error_handler* const error_handlerp,  // de-referenced
    struct rawrtc_data_channel* const channel) {
    // Check arguments
    if (!error_handlerp || !channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Get error handler (if any)
    if (channel->error_handler) {
        *error_handlerp = channel->error_handler;
        return RAWRTC_CODE_SUCCESS;
    } else {
        return RAWRTC_CODE_NO_VALUE;
    }
}

/*
 * Set the data channel's close handler.
 */
enum rawrtc_code rawrtc_data_channel_set_close_handler(
    struct rawrtc_data_channel* const channel,
    rawrtc_data_channel_close_handler const close_handler  // nullable
) {
    // Check arguments
    if (!channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set close handler & done
    channel->close_handler = close_handler;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the data channel's close handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_data_channel_get_close_handler(
    rawrtc_data_channel_close_handler* const close_handlerp,  // de-referenced
    struct rawrtc_data_channel* const channel) {
    // Check arguments
    if (!close_handlerp || !channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Get close handler (if any)
    if (channel->close_handler) {
        *close_handlerp = channel->close_handler;
        return RAWRTC_CODE_SUCCESS;
    } else {
        return RAWRTC_CODE_NO_VALUE;
    }
}

/*
 * Set the data channel's message handler.
 */
enum rawrtc_code rawrtc_data_channel_set_message_handler(
    struct rawrtc_data_channel* const channel,
    rawrtc_data_channel_message_handler const message_handler  // nullable
) {
    // Check arguments
    if (!channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set message handler & done
    channel->message_handler = message_handler;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Get the data channel's message handler.
 * Returns `RAWRTC_CODE_NO_VALUE` in case no handler has been set.
 */
enum rawrtc_code rawrtc_data_channel_get_message_handler(
    rawrtc_data_channel_message_handler* const message_handlerp,  // de-referenced
    struct rawrtc_data_channel* const channel) {
    // Check arguments
    if (!message_handlerp || !channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Get message handler (if any)
    if (channel->message_handler) {
        *message_handlerp = channel->message_handler;
        return RAWRTC_CODE_SUCCESS;
    } else {
        return RAWRTC_CODE_NO_VALUE;
    }
}
