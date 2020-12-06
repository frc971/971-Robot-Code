#!/usr/bin/python3

# Application to generate C++ code with a binary flatbuffer file embedded in it
# as a Span.

import sys
from pathlib import Path


def main(argv):
    if len(argv) != 4:
        return 1

    input_path = sys.argv[1]
    output_path = sys.argv[2]
    function = sys.argv[3].split("::")
    include_guard = output_path.replace('/', '_').replace('-', '_').replace(
        '.', '_').upper() + '_'

    output_prefix = [
        b'#ifndef ' + include_guard.encode(),
        b'#define ' + include_guard.encode(),
        b'',
        b'#include "absl/types/span.h"',
        b'',
    ]

    for f in function[:-1]:
        output_prefix.append(b'namespace ' + f.encode() + b' {')

    output_prefix.append(b'')
    output_prefix.append(b'inline absl::Span<const uint8_t> ' +
                         function[-1].encode() + b'() {')

    output_suffix = [
        b'  return absl::Span<const uint8_t>(reinterpret_cast<const uint8_t*>(kData), sizeof(kData));',
        b'}'
    ]
    output_suffix.append(b'')

    for f in function[:-1]:
        output_suffix.append(b'}  // namespace ' + f.encode())

    output_suffix.append(b'')
    output_suffix.append(b'#endif  // ' + include_guard.encode())

    with open(input_path, 'rb') as binary_file:
        bfbs = binary_file.read()

    # Write out the header file
    with open(output_path, 'wb') as output:
        for line in output_prefix:
            output.write(line)
            output.write(b'\n')
        output.write(b'  alignas(64) static constexpr char kData[] = "')
        for byte in bfbs:
            output.write(b'\\x' + (b'%x' % byte).zfill(2))
        output.write(b'";\n')
        for line in output_suffix:
            output.write(line)
            output.write(b'\n')


if __name__ == '__main__':
    sys.exit(main(sys.argv))
