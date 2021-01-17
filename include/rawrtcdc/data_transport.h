#pragma once
#include <rawrtcc/code.h>
#include <re.h>

/**
 * Data transport type.
 */
enum rawrtc_data_transport_type {
    RAWRTC_DATA_TRANSPORT_TYPE_SCTP,
};

/**
 * Generic data transport.
 */
struct rawrtc_data_transport;

/**
 * Get the data transport's type and underlying transport reference.
 * `*internal_transportp` must be unreferenced.
 */
enum rawrtc_code rawrtc_data_transport_get_transport(
    enum rawrtc_data_transport_type* const typep,  // de-referenced
    void** const internal_transportp,  // de-referenced
    struct rawrtc_data_transport* const transport);

/**
 * Translate a data transport type to str.
 */
char const* rawrtc_data_transport_type_to_str(enum rawrtc_data_transport_type const type);
