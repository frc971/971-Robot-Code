# This file contains replacements for select where the keys have more abstract
# meanings so we can map multiple conditions to the same value easily and
# quickly find issues where something new isn't handled.
# It will also make adding ORs when it makes sense easy to do nicely.

all_cpus = ['amd64', 'roborio']
'''All of the CPUs we know about.'''

'''A select wrapper for CPU architectures.

Args:
  values: A mapping from architecture names (as strings) to other things.
          Currently amd64 and roborio are recognized.
          'else' is also allowed as a default.
Returns a select which evaluates to the correct element of values.
'''
def cpu_select(values):
  for cpu in all_cpus:
    if cpu not in values:
      if 'else' in values:
        values[cpu] = values['else']
      else:
        fail('Need to handle %s CPUs!' % cpu, 'values')
  for key in values:
    if key not in all_cpus and key != 'else':
      fail('Not sure what a %s CPU is!' % key, 'values')
  return select({
    '//tools:cpu_k8': values['amd64'],
    '//tools:cpu_roborio': values['roborio'],
  })

'''A select wrapper for address space sizes.

Args:
  values: A mapping from address space sizes (as strings) to other things.
Returns a select which evaluates to the correct element of values.
'''
def address_size_select(values):
  if '32' not in values:
    fail('Need to handle 32 bit addresses!', 'values')
  if '64' not in values:
    fail('Need to handle 64 bit addresses!', 'values')
  return select({
    '//tools:cpu_k8': values['64'],
    '//tools:cpu_roborio': values['32'],
  })

'''A select wrapper for compilers.

Args:
  values: A mapping from compiler names (as strings) to other things.
          Currently 'gcc' and 'clang' are recognized.
Returns a select which evaluates to the correct element of values.
'''
def compiler_select(values):
  if 'gcc' not in values:
    fail('Need to handle gcc!', 'values')
  if 'clang' not in values:
    fail('Need to handle clang!', 'values')
  return select({
    '//tools:compiler_gcc': values['gcc'],
    '//tools:compiler_clang': values['clang'],
  })
