# Generate configuration header
configure_file(
    input: 'config.h.in',
    output: 'config.h',
    configuration: configuration,
    install_dir: '/'.join([get_option('includedir'), 'rawrtcdc']))

# Install headers
includes = files([
    'data_channel.h',
    'data_channel_parameters.h',
    'data_transport.h',
    'external.h',
    'main.h',
    'sctp_capabilities.h',
    'sctp_transport.h',
])
if get_option('sctp_redirect_transport')
    includes += files('sctp_redirect_transport.h')
endif
install_headers(includes, subdir: 'rawrtcdc')
