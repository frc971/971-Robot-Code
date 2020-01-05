from __future__ import print_function

import sys

header_in_name = sys.argv[1]
instances_txt_name = sys.argv[2]
resource_type_txt_name = sys.argv[3]
header_out_name = sys.argv[4]

with open(header_in_name, 'r') as f:
  header_in = f.read().replace('\r', '')

with open(instances_txt_name, 'r') as f:
  instances_txt = [l.strip() for l in f.readlines()]

with open(resource_type_txt_name, 'r') as f:
  resource_type_txt = [l.strip() for l in f.readlines()]

with open(header_out_name, 'w') as out:
  header = header_in
  header = header.replace('${usage_reporting_types_cpp}',
                          ',\n'.join(resource_type_txt))
  header = header.replace('${usage_reporting_instances_cpp}',
                          ',\n'.join(instances_txt))
  out.write(header)
