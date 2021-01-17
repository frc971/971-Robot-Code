#include "channel.h"
#include "../data_channel_parameters/parameters.h"
#include "../data_transport/transport.h"
#include <rawrtcdc/config.h>
#include <rawrtcdc/data_channel.h>
#include <rawrtcc/code.h>
#include <re.h>

#define DEBUG_MODULE "data-channel"
//#define RAWRTC_DEBUG_MODULE_LEVEL 7 // Note: Uncomment this to debug this module only
#include <rawrtcc/debug.h>

/*
 * Change the state of the data channel.
 * Will call the corresponding handler.
 * Caller MUST ensure that the same state is not set twice.
 */
void rawrtc_data_channel_set_state(
    struct rawrtc_data_channel* const channel,  // not checked
    enum rawrtc_data_channel_state const state) {
    // Set state
    // Note: Keep this here as it will prevent infinite recursion during closing/destroying
    channel->state = state;
    DEBUG_PRINTF(
        "Data channel '%s' state changed to %s\n",
        channel->parameters->label ? channel->parameters->label : "n/a",
        rawrtc_data_channel_state_to_name(state));

    // TODO: Clear options flag?

    // Call transport handler (if any) and user application handler
    switch (state) {
        case RAWRTC_DATA_CHANNEL_STATE_OPEN:
            // Call handler
            if (channel->open_handler) {
                channel->open_handler(channel->arg);
            }
            break;

        case RAWRTC_DATA_CHANNEL_STATE_CLOSING:
            // Nothing to do.
            break;

        case RAWRTC_DATA_CHANNEL_STATE_CLOSED:
            // Call handler
            if (channel->close_handler) {
                channel->close_handler(channel->arg);
            }
            break;
        default:
            break;
    }
}

/*
 * Destructor for an existing data channel.
 */
static void rawrtc_data_channel_destroy(void* arg) {
    struct rawrtc_data_channel* const channel = arg;

    // Unset all handlers
    rawrtc_data_channel_unset_handlers(channel);

    // Close channel
    // Note: The function will ensure that the channel is not closed before it's initialised
    rawrtc_data_channel_close(channel);

    // Un-reference
    mem_deref(channel->transport);
    mem_deref(channel->transport_arg);
    mem_deref(channel->parameters);
}

/*
 * Create a data channel (internal).
 */
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
    bool const call_handler) {
    enum rawrtc_code error;
    struct rawrtc_data_channel* channel;

    // Check arguments
    if (!channelp || !transport || !parameters) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Allocate
    channel = mem_zalloc(sizeof(*channel), rawrtc_data_channel_destroy);
    if (!channel) {
        return RAWRTC_CODE_NO_MEMORY;
    }

    // Set fields/reference
    channel->flags = 0;
    channel->state = RAWRTC_DATA_CHANNEL_STATE_CONNECTING;
    channel->transport = mem_ref(transport);
    channel->parameters = mem_ref(parameters);
    channel->open_handler = open_handler;
    channel->buffered_amount_low_handler = buffered_amount_low_handler;
    channel->error_handler = error_handler;
    channel->close_handler = close_handler;
    channel->message_handler = message_handler;
    channel->arg = arg;

    // Create data channel on transport
    if (call_handler) {
        error = transport->channel_create(transport, channel, parameters);
        if (error) {
            goto out;
        }
    } else {
        error = RAWRTC_CODE_SUCCESS;
    }

    // Done
    DEBUG_PRINTF(
        "Created data channel: label=%s, type=%d, reliability-parameter=%" PRIu32 ", "
        "protocol=%s, negotiated=%s, id=%" PRIu16 "\n",
        parameters->label ? parameters->label : "n/a", parameters->channel_type,
        parameters->reliability_parameter, parameters->protocol ? parameters->protocol : "n/a",
        parameters->negotiated ? "yes" : "no", parameters->id);

out:
    if (error) {
        mem_deref(channel);
    } else {
        // Update flags & set pointer
        channel->flags |= RAWRTC_DATA_CHANNEL_FLAGS_INITIALIZED;
        *channelp = channel;
    }
    return error;
}

/*
 * Call the data channel handler (internal).
 *
 * Important: Data transport implementations SHALL call this function
 * instead of calling the channel handler directly.
 */
void rawrtc_data_channel_call_channel_handler(
    struct rawrtc_data_channel* const channel,  // not checked
    rawrtc_data_channel_handler const channel_handler,  // nullable
    void* const arg) {
    // Call handler (if any)
    if (channel_handler) {
        channel_handler(channel, arg);
    }
}

/*
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
) {
    enum rawrtc_code const error = rawrtc_data_channel_create_internal(
        channelp, transport, parameters, open_handler, buffered_amount_low_handler, error_handler,
        close_handler, message_handler, arg, true);

    // Done
    return error;
}

/*
 * Set the argument of a data channel that is passed to the various
 * handlers.
 */
enum rawrtc_code rawrtc_data_channel_set_arg(
    struct rawrtc_data_channel* const channel,
    void* const arg  // nullable
) {
    // Check arguments
    if (!channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set handler argument & done
    channel->arg = arg;
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Send data via the data channel.
 */
enum rawrtc_code rawrtc_data_channel_send(
    struct rawrtc_data_channel* const channel,
    struct mbuf* const buffer,  // nullable (if empty message), referenced
    bool const is_binary) {
    // Check arguments
    if (!channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Check state
    // TODO: Is this correct or can we send during `connecting` as well?
    if (channel->state != RAWRTC_DATA_CHANNEL_STATE_OPEN) {
        return RAWRTC_CODE_INVALID_STATE;
    }

    // Call handler
    return channel->transport->channel_send(channel, buffer, is_binary);
}

/*
 * Close the data channel.
 */
enum rawrtc_code rawrtc_data_channel_close(struct rawrtc_data_channel* const channel) {
    // Check arguments
    if (!channel) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Don't close before the channel is initialised
    // Note: This is needed as this function may be called in the destructor of the data channel
    if (!(channel->flags & RAWRTC_DATA_CHANNEL_FLAGS_INITIALIZED)) {
        return RAWRTC_CODE_SUCCESS;
    }

    // Check state
    if (channel->state == RAWRTC_DATA_CHANNEL_STATE_CLOSING ||
        channel->state == RAWRTC_DATA_CHANNEL_STATE_CLOSED) {
        return RAWRTC_CODE_SUCCESS;
    }

    // Close channel
    DEBUG_PRINTF("Closing data channel: %s\n", channel->parameters->label);
    return channel->transport->channel_close(channel);
}
