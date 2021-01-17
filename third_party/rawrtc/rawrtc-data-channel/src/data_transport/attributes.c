#include "transport.h"
#include <rawrtcdc/data_transport.h>
#include <rawrtcc/code.h>
#include <re.h>

/**
 * Get the data transport's type and underlying transport reference.
 * `*internal_transportp` must be unreferenced.
 */
enum rawrtc_code rawrtc_data_transport_get_transport(
    enum rawrtc_data_transport_type* const typep,  // de-referenced
    void** const internal_transportp,  // de-referenced
    struct rawrtc_data_transport* const transport) {
    // Check arguments
    if (!typep || !transport) {
        return RAWRTC_CODE_INVALID_ARGUMENT;
    }

    // Set type & transport
    *typep = transport->type;
    *internal_transportp = mem_ref(transport->transport);

    // Done
    return RAWRTC_CODE_SUCCESS;
}
