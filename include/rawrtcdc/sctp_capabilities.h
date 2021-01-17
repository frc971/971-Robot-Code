#pragma once
#include <rawrtcc/code.h>
#include <re.h>

/**
 * SCTP capabilities.
 */
struct rawrtc_sctp_capabilities;

/**
 * Create a new SCTP transport capabilities instance.
 * `*capabilitiesp` must be unreferenced.
 */
enum rawrtc_code rawrtc_sctp_capabilities_create(
    struct rawrtc_sctp_capabilities** const capabilitiesp,  // de-referenced
    uint64_t const max_message_size);

/**
 * Get the SCTP parameter's maximum message size value.
 *
 * Note: A value of `0` indicates that the implementation supports
 *       receiving messages of arbitrary size.
 */
enum rawrtc_code rawrtc_sctp_capabilities_get_max_message_size(
    uint64_t* const max_message_sizep,  // de-referenced
    struct rawrtc_sctp_capabilities* const capabilities);

/**
 * Print debug information for SCTP capabilities.
 */
int rawrtc_sctp_capabilities_debug(
    struct re_printf* const pf, struct rawrtc_sctp_capabilities const* const capabilities);
