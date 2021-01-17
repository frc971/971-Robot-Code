#include <rawrtcdc/data_transport.h>

/*
 * Translate a data transport type to str.
 */
char const* rawrtc_data_transport_type_to_str(enum rawrtc_data_transport_type const type) {
    switch (type) {
        case RAWRTC_DATA_TRANSPORT_TYPE_SCTP:
            return "SCTP";
        default:
            return "???";
    }
}
