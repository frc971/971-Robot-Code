disable_gtk_binaries = True

def gtk_dependent_cc_library(**kwargs):
  if not disable_gtk_binaries:
    native.cc_library(**kwargs)

def gtk_dependent_cc_binary(**kwargs):
  if not disable_gtk_binaries:
    native.cc_binary(**kwargs)
