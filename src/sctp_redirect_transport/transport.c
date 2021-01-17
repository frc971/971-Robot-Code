#include "transport.h"
#include "../crc32c/crc32c.h"
#include <rawrtcdc/config.h>
#include <rawrtcdc/external.h>
#include <rawrtcdc/sctp_redirect_transport.h>
#include <rawrtcdc/sctp_transport.h>
#include <rawrtcc/code.h>
#include <rawrtcc/utils.h>
#include <re.h>
#include <errno.h>
#include <netinet/in.h>  // IPPROTO_RAW, ntohs, htons
#include <string.h>  // memset
#include <sys/socket.h>  // AF_INET, SOCK_RAW, sendto, recvfrom
#include <sys/types.h>
#include <unistd.h>  // close

#define DEBUG_MODULE "redirect-transport"
//#define RAWRTC_DEBUG_MODULE_LEVEL 7 // Note: Uncomment this to debug this module only
#include <rawrtcc/debug.h>

/*
 * Patch local and remote port in the SCTP packet header.
 */
static void patch_sctp_header(
    struct mbuf* const buffer, uint16_t const source, uint16_t const destination) {
    size_t const start = buffer->pos;
    int err;
    uint32_t checksum;

    // Patch source port
    err = mbuf_write_u16(buffer, htons(source));
    if (err) {
        DEBUG_WARNING("Could not patch source port, reason: %m\n", err);
        return;
    }

    // Patch destination port
    err = mbuf_write_u16(buffer, htons(destination));
    if (err) {
        DEBUG_WARNING("Could not patch destination port, reason: %m\n", err);
        return;
    }

    // Skip verification tag
    mbuf_advance(buffer, 4);

    // Reset checksum field to '0' and rewind back
    memset(mbuf_buf(buffer), 0, 4);
    mbuf_set_pos(buffer, start);
    // Recalculate checksum
    checksum = rawrtc_crc32c(mbuf_buf(buffer), mbuf_get_left(buffer));
    // Advance to checksum field, set it and rewind back
    mbuf_advance(buffer, 8);
    err = mbuf_write_u32(buffer, checksum);
    if (err) {
        DEBUG_WARNING("Could not patch checksum, reason: %m\n", err);
        return;
    }
    mbuf_set_pos(buffer, start);
}

/*
 * Handle outgoing messages (that came in from the raw socket).
 */
static void redirect_from_raw(
    int flags,
    void* arg  // not checked
) {
    struct rawrtc_sctp_redirect_transport* const transport = arg;
    struct mbuf* const buffer = transport->buffer;
    enum rawrtc_code error;
    struct sockaddr_in from_address;
    socklen_t address_length;
    ssize_t length;
    struct sa from = {0};
    size_t header_length;
    uint16_t source;
    uint16_t destination;

    // Detached?
    if (transport->flags & RAWRTC_SCTP_REDIRECT_TRANSPORT_FLAGS_DETACHED) {
        DEBUG_PRINTF("Ignoring SCTP packet ready event, transport is detached\n");
        return;
    }

    if ((flags & FD_READ) == FD_READ) {
        // Receive
        address_length = sizeof(from_address);
        length = recvfrom(
            transport->socket, mbuf_buf(buffer), mbuf_get_space(buffer), 0,
            (struct sockaddr*) &from_address, &address_length);
        if (length == -1) {
            switch (errno) {
                case EAGAIN:
#if (defined(EWOULDBLOCK) && EAGAIN != EWOULDBLOCK)
                case EWOULDBLOCK:
#endif
                    break;
                default:
                    DEBUG_WARNING("Unable to receive raw message: %m\n", errno);
                    break;
            }
            goto out;
        }
        mbuf_set_end(buffer, (size_t) length);

        // Check address
        error = rawrtc_error_to_code(sa_set_sa(&from, (struct sockaddr*) &from_address));
        if (error) {
            DEBUG_WARNING("Invalid sender address: %m\n", error);
            goto out;
        }
        DEBUG_PRINTF("Received %zu bytes via RAW from %j\n", mbuf_get_left(buffer), &from);
        if (!sa_isset(&from, SA_ADDR) && !sa_cmp(&transport->redirect_address, &from, SA_ADDR)) {
            DEBUG_WARNING("Ignoring data from unknown address");
            goto out;
        }

        // Skip IPv4 header
        header_length = (size_t)(mbuf_read_u8(buffer) & 0xf);
        mbuf_advance(buffer, -1);
        DEBUG_PRINTF("IPv4 header length: %zu\n", header_length);
        mbuf_advance(buffer, header_length * 4);

        // Read source and destination port
        source = ntohs(mbuf_read_u16(buffer));
        destination = ntohs(mbuf_read_u16(buffer));
        sa_set_port(&from, source);
        (void) destination;
        DEBUG_PRINTF("Raw from %J to %" PRIu16 "\n", &from, destination);
        mbuf_advance(buffer, -4);

        // Is this from the correct source?
        if (source != sa_port(&transport->redirect_address)) {
            DEBUG_PRINTF("Ignored data from different source port %" PRIu16 "\n", source);
            goto out;
        }

        // Update SCTP header with changed ports
        patch_sctp_header(buffer, transport->local_port, transport->remote_port);

        // Pass to outbound handler
        error = transport->context.outbound_handler(buffer, 0x00, 0x00, transport->context.arg);
        if (error) {
            DEBUG_WARNING("Could not send packet, reason: %s\n", rawrtc_code_to_str(error));
            goto out;
        }
    }

out:
    // Rewind buffer
    mbuf_rewind(buffer);
}

/*
 * Change the state of the SCTP redirect transport.
 * Caller MUST ensure that the same state is not set twice.
 */
static void set_state(
    struct rawrtc_sctp_redirect_transport* const transport,  // not checked
    enum rawrtc_sctp_redirect_transport_state const state) {
    // Closed?
    if (state == RAWRTC_SCTP_REDIRECT_TRANSPORT_STATE_CLOSED) {
        // Stop listening and close raw socket
        if (transport->socket != -1) {
            fd_close(transport->socket);
            if (close(transport->socket)) {
                DEBUG_WARNING("Closing raw socket failed: %m\n", errno);
            }
        }

        // Mark as detached & detach from DTLS transport
        transport->flags |= RAWRTC_SCTP_REDIRECT_TRANSPORT_FLAGS_DETACHED;
        if (transport->context.detach_handler) {
            transport->context.detach_handler(transport->context.arg);
        }
    }

    // Set state
    transport->state = state;

    // Call handler (if any)
    if (transport->state_change_handler) {
        transport->state_change_handler(state, transport->arg);
    }
}

static enum rawrtc_code validate_context(
    struct rawrtc_sctp_transport_context* const context  // not checked
) {
    // Ensure the context contains all necessary callbacks
    if (!context->state_getter || !context->outbound_handler || !context->detach_handler) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    } else {
        return RAWRTC_CODE_SUCCESS;
    }
}

/*
 * Destructor for an existing SCTP redirect transport.
 */
static void rawrtc_sctp_redirect_transport_destroy(void* arg) {
    struct rawrtc_sctp_redirect_transport* const transport = arg;

    // Stop transport
    rawrtc_sctp_redirect_transport_stop(transport);

    // Call 'destroyed' handler (if fully initialised)
    if (transport->flags & RAWRTC_SCTP_REDIRECT_TRANSPORT_FLAGS_INITIALIZED &&
        transport->context.destroyed_handler) {
        transport->context.destroyed_handler(transport->context.arg);
    }

    // Un-reference
    mem_deref(transport->buffer);
}

/*
 * Create an SCTP redirect transport from an external DTLS transport.
 * `*transportp` must be unreferenced.
 *
 * `redirect_ip` must be a IPv4 address.
 *
 * Important: The redirect transport requires to be run inside re's
 *            event loop (`re_main`).
 */
enum rawrtc_code rawrtc_sctp_redirect_transport_create_from_external(
    struct rawrtc_sctp_redirect_transport** const transportp,  // de-referenced
    struct rawrtc_sctp_transport_context* const context,  // copied
    uint16_t const port,  // zeroable
    char* const redirect_ip,  // copied
    uint16_t const redirect_port,
    rawrtc_sctp_redirect_transport_state_change_handler const state_change_handler,  // nullable
    void* const arg  // nullable
) {
    enum rawrtc_code error;
    struct sa redirect_address;
    enum rawrtc_external_dtls_transport_state dtls_transport_state;
    struct rawrtc_sctp_redirect_transport* transport;

    // Check arguments
    if (!transportp || !context || !redirect_ip || redirect_port == 0) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Ensure it's an IPv4 address
    error = rawrtc_error_to_code(sa_set_str(&redirect_address, redirect_ip, redirect_port));
    if (error) {
        return error;
    }
    if (sa_af(&redirect_address) != AF_INET) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Ensure the context contains all necessary callbacks
    error = validate_context(context);
    if (error) {
        return error;
    }

    // Check DTLS transport state
    error = context->state_getter(&dtls_transport_state, context->arg);
    if (error) {
        DEBUG_WARNING(
            "Getting external DTLS transport state failed: %s\n", rawrtc_code_to_str(error));
        return RAWRTC_CODE_EXTERNAL_ERROR;
    }
    if (dtls_transport_state == RAWRTC_EXTERNAL_DTLS_TRANSPORT_STATE_CLOSED_OR_FAILED) {
        return RAWRTC_CODE_INVALID_STATE;
    }

    // Allocate
    transport = mem_zalloc(sizeof(*transport), rawrtc_sctp_redirect_transport_destroy);
    if (!transport) {
        return RAWRTC_CODE_NO_MEMORY;
    }

    // Set fields
    transport->context = *context;
    transport->flags = 0;
    transport->state = RAWRTC_SCTP_REDIRECT_TRANSPORT_STATE_NEW;
    transport->local_port = port > 0 ? port : RAWRTC_SCTP_REDIRECT_TRANSPORT_DEFAULT_PORT;
    transport->redirect_address = redirect_address;
    transport->state_change_handler = state_change_handler;
    transport->arg = arg;

    // Create buffer
    transport->buffer = mbuf_alloc(RAWRTC_SCTP_REDIRECT_TRANSPORT_RAW_SOCKET_RECEIVE_SIZE);
    if (!transport->buffer) {
        error = RAWRTC_CODE_NO_MEMORY;
        goto out;
    }

    // Create raw socket
    transport->socket = socket(AF_INET, SOCK_RAW, IPPROTO_SCTP);
    if (transport->socket == -1) {
        error = rawrtc_error_to_code(errno);
        goto out;
    }

    // Set non-blocking
    error = rawrtc_error_to_code(net_sockopt_blocking_set(transport->socket, false));
    if (error) {
        goto out;
    }

out:
    if (error) {
        mem_deref(transport);
    } else {
        // Set pointer & mark as initialised
        *transportp = transport;
        transport->flags |= RAWRTC_SCTP_REDIRECT_TRANSPORT_FLAGS_INITIALIZED;
    }
    return error;
}

/*
 * Start an SCTP redirect transport.
 */
enum rawrtc_code rawrtc_sctp_redirect_transport_start(
    struct rawrtc_sctp_redirect_transport* const transport,
    struct rawrtc_sctp_capabilities const* const remote_capabilities,  // copied
    uint16_t remote_port  // zeroable
) {
    enum rawrtc_code error;

    // Check arguments
    if (!transport || !remote_capabilities) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Check state
    if (transport->state != RAWRTC_SCTP_REDIRECT_TRANSPORT_STATE_NEW) {
        return RAWRTC_CODE_INVALID_STATE;
    }

    // Set default port (if 0)
    if (remote_port == 0) {
        remote_port = transport->local_port;
    }

    // Store remote port
    transport->remote_port = remote_port;

    // Listen on raw socket
    error =
        rawrtc_error_to_code(fd_listen(transport->socket, FD_READ, redirect_from_raw, transport));
    if (error) {
        goto out;
    }

    // Update state & done
    set_state(transport, RAWRTC_SCTP_REDIRECT_TRANSPORT_STATE_OPEN);
    error = RAWRTC_CODE_SUCCESS;

out:
    if (error) {
        // Stop listening on raw socket
        fd_close(transport->socket);
    }
    return error;
}

/*
 * Stop and close the SCTP redirect transport.
 */
enum rawrtc_code rawrtc_sctp_redirect_transport_stop(
    struct rawrtc_sctp_redirect_transport* const transport) {
    // Check arguments
    if (!transport) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Check state
    if (transport->flags & RAWRTC_SCTP_REDIRECT_TRANSPORT_FLAGS_DETACHED ||
        transport->state == RAWRTC_SCTP_REDIRECT_TRANSPORT_STATE_CLOSED) {
        return RAWRTC_CODE_SUCCESS;
    }

    // Update state
    set_state(transport, RAWRTC_SCTP_REDIRECT_TRANSPORT_STATE_CLOSED);
    return RAWRTC_CODE_SUCCESS;
}

/*
 * Feed inbound data to the SCTP redirect transport (that will be sent
 * out via the raw socket).
 *
 * `buffer` contains the data to be fed to the raw transport. Since
 * the data is not going to be referenced, you can pass a *fake* `mbuf`
 * structure that hasn't been allocated with `mbuf_alloc` to avoid
 * copying.
 *
 * Return `RAWRTC_CODE_INVALID_STATE` in case the transport is closed.
 * In case the buffer could not be sent due to the raw socket's buffer
 * being too full, `RAWRTC_CODE_TRY_AGAIN_LATER` will be returned. You
 * can safely ignore this code since SCTP will retransmit data on a
 * reliable stream.
 * Otherwise, `RAWRTC_CODE_SUCCESS` is being returned.
 */
enum rawrtc_code rawrtc_sctp_redirect_transport_feed_inbound(
    struct rawrtc_sctp_redirect_transport* const transport, struct mbuf* const buffer) {
    ssize_t length;

    // Check arguments
    if (!transport || !buffer) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Detached?
    if (transport->flags & RAWRTC_SCTP_REDIRECT_TRANSPORT_FLAGS_DETACHED) {
        return RAWRTC_CODE_INVALID_STATE;
    }

    // Check state
    if (transport->state != RAWRTC_SCTP_REDIRECT_TRANSPORT_STATE_OPEN) {
        // Note: Silently ignore inbound packets received as long as the transport is not open.
        return RAWRTC_CODE_SUCCESS;
    }

    // Update SCTP header with changed ports
    patch_sctp_header(buffer, transport->local_port, sa_port(&transport->redirect_address));

    // Send over raw socket
    DEBUG_PRINTF(
        "Redirecting message (%zu bytes) to %J\n", mbuf_get_left(buffer),
        &transport->redirect_address);
    length = sendto(
        transport->socket, mbuf_buf(buffer), mbuf_get_left(buffer), 0,
        &transport->redirect_address.u.sa, transport->redirect_address.len);
    if (length == -1) {
        switch (errno) {
            case EAGAIN:
#if (defined(EWOULDBLOCK) && EAGAIN != EWOULDBLOCK)
            case EWOULDBLOCK:
#endif
                // We're just dropping the packets in this case. SCTP will retransmit eventually.
                return RAWRTC_CODE_TRY_AGAIN_LATER;
            default:
                DEBUG_WARNING("Unable to redirect message: %m\n", errno);
                return rawrtc_error_to_code(errno);
        }
    }

    // Advance buffer & done
    mbuf_advance(buffer, length);
    return RAWRTC_CODE_SUCCESS;
}
