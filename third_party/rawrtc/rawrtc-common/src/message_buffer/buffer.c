#include <rawrtcc/code.h>
#include <rawrtcc/message_buffer.h>
#include <re.h>

/*
 * Message buffer.
 */
struct buffered_message {
    struct le le;
    struct mbuf* buffer;  // referenced
    void* context;  // referenced, nullable
};

/*
 * Destructor for an existing buffered message.
 */
static void rawrtc_message_buffer_destroy(void* arg) {
    struct buffered_message* const buffered_message = arg;

    // Un-reference
    mem_deref(buffered_message->context);
    mem_deref(buffered_message->buffer);
}

/*
 * Create a message buffer and add it to a list.
 *
 * TODO: Add timestamp to be able to ignore old messages
 */
enum rawrtc_code rawrtc_message_buffer_append(
    struct list* const message_buffer,
    struct mbuf* const buffer,  // referenced
    void* const context  // referenced, nullable
) {
    struct buffered_message* buffered_message;

    // Check arguments
    if (!message_buffer || !buffer) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Create buffered message
    buffered_message = mem_zalloc(sizeof(*buffered_message), rawrtc_message_buffer_destroy);
    if (!buffered_message) {
        return RAWRTC_CODE_NO_MEMORY;
    }

    // Set fields
    buffered_message->buffer = mem_ref(buffer);
    buffered_message->context = mem_ref(context);

    // Add to list
    list_append(message_buffer, &buffered_message->le, buffered_message);
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Apply a receive handler to buffered messages.
 *
 * Will stop iterating and return `RAWRTC_CODE_STOP_ITERATION` in case
 * the message handler returned `false`.
 */
enum rawrtc_code rawrtc_message_buffer_clear(
    struct list* const message_buffer,
    rawrtc_message_buffer_handler* const message_handler,
    void* arg) {
    struct le* le;
    bool unlink;

    // Check arguments
    if (!message_buffer || !message_handler) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Handle each message
    le = list_head(message_buffer);
    while (le != NULL) {
        struct buffered_message* const buffered_message = le->data;

        // Handle message
        unlink = message_handler(buffered_message->buffer, buffered_message->context, arg);
        struct le* next = le->next;
        if (unlink) {
            list_unlink(le);
        }

        // Get next message
        le = next;

        // Remove message
        if (unlink) {
            mem_deref(buffered_message);
        } else {
            return RAWRTC_CODE_STOP_ITERATION;
        }
    }

    // Done
    return RAWRTC_CODE_SUCCESS;
}
