#pragma once
#include "data_channel.h"
#include <rawrtcc/code.h>
#include <re.h>

/**
 * Data channel parameters.
 */
struct rawrtc_data_channel_parameters;

/**
 * Create data channel parameters.
 *
 * For `RAWRTC_DATA_CHANNEL_TYPE_RELIABLE_*`, the reliability parameter
 * is being ignored.
 *
 * When using `RAWRTC_DATA_CHANNEL_TYPE_*_RETRANSMIT`, the reliability
 * parameter specifies the number of times a retransmission occurs if
 * not acknowledged before the message is being discarded.
 *
 * When using `RAWRTC_DATA_CHANNEL_TYPE_*_TIMED`, the reliability
 * parameter specifies the time window in milliseconds during which
 * (re-)transmissions may occur before the message is being discarded.
 *
 * `*parametersp` must be unreferenced.
 *
 * In case `negotiated` is set to `false`, the `id` is being ignored.
 */
enum rawrtc_code rawrtc_data_channel_parameters_create(
    struct rawrtc_data_channel_parameters** const parametersp,  // de-referenced
    char const* const label,  // copied, nullable
    enum rawrtc_data_channel_type const channel_type,
    uint32_t const reliability_parameter,
    char const* const protocol,  // copied, nullable
    bool const negotiated,
    uint16_t const id);

/**
 * Get the label from the data channel parameters.
 * `*labelp` will be set to a copy of the parameter's label and must be
 * unreferenced.
 *
 * Return `RAWRTC_CODE_NO_VALUE` in case no label has been set.
 * Otherwise, `RAWRTC_CODE_SUCCESS` will be returned and `*parameters*
 * must be unreferenced.
 */
enum rawrtc_code rawrtc_data_channel_parameters_get_label(
    char** const labelp,  // de-referenced
    struct rawrtc_data_channel_parameters* const parameters);

/**
 * Get the channel type from the data channel parameters.
 */
enum rawrtc_code rawrtc_data_channel_parameters_get_channel_type(
    enum rawrtc_data_channel_type* const channel_typep,  // de-referenced
    struct rawrtc_data_channel_parameters* const parameters);

/**
 * Get the reliability parameter from the data channel parameters.
 *
 * Return `RAWRTC_CODE_NO_VALUE` in case the channel type is
 * `RAWRTC_DATA_CHANNEL_TYPE_RELIABLE_*`. Otherwise,
 * `RAWRTC_CODE_SUCCESS` will be returned.
 */
enum rawrtc_code rawrtc_data_channel_parameters_get_reliability_parameter(
    uint32_t* const reliability_parameterp,  // de-referenced
    struct rawrtc_data_channel_parameters* const parameters);

/**
 * Get the protocol from the data channel parameters.
 * `*protocolp` will be set to a copy of the parameter's protocol and
 * must be unreferenced.
 *
 * Return `RAWRTC_CODE_NO_VALUE` in case no protocol has been set.
 * Otherwise, `RAWRTC_CODE_SUCCESS` will be returned and `*protocolp*
 * must be unreferenced.
 */
enum rawrtc_code rawrtc_data_channel_parameters_get_protocol(
    char** const protocolp,  // de-referenced
    struct rawrtc_data_channel_parameters* const parameters);

/**
 * Get the 'negotiated' flag from the data channel parameters.
 */
enum rawrtc_code rawrtc_data_channel_parameters_get_negotiated(
    bool* const negotiatedp,  // de-referenced
    struct rawrtc_data_channel_parameters* const parameters);

/**
 * Get the negotiated id from the data channel parameters.
 *
 * Return `RAWRTC_CODE_NO_VALUE` in case the 'negotiated' flag is set
 * `false`. Otherwise, `RAWRTC_CODE_SUCCESS` will be returned.
 */
enum rawrtc_code rawrtc_data_channel_parameters_get_id(
    uint16_t* const idp,  // de-referenced
    struct rawrtc_data_channel_parameters* const parameters);
