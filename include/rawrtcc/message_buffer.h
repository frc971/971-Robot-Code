#pragma once
#include "code.h"
#include <re.h>

/// Handler for buffered messages.
///
/// Return `true` if the message has been handled successfully and can
/// be unlinked, `false` to stop processing messages and keep the current
/// message in the list.
typedef bool(rawrtc_message_buffer_handler)(
    struct mbuf* const buffer, void* const context, void* const arg);

/// Create a message buffer and add it to a list.
enum rawrtc_code rawrtc_message_buffer_append(
    struct list* const message_buffer,
    struct mbuf* const buffer,  // referenced
    void* const context  // referenced, nullable
);

/// Apply a receive handler to buffered messages.
///
/// Will stop iterating and return `RAWRTC_CODE_STOP_ITERATION` in case
/// the message handler returned `false`.
enum rawrtc_code rawrtc_message_buffer_clear(
    struct list* const message_buffer,
    rawrtc_message_buffer_handler* const message_handler,
    void* arg);
