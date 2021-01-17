# Generate configuration header
configure_file(
    input: 'config.h.in',
    output: 'config.h',
    configuration: configuration,
    install_dir: '/'.join([get_option('includedir'), 'rawrtcc']))

# Install headers
includes = files([
    'code.h',
    'debug.h',
    'message_buffer.h',
    'utils.h',
])
install_headers(includes, subdir: 'rawrtcc')
