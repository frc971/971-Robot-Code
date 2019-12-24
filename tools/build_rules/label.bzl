# This file provides any necessary label abstractions since it doesn't seem like
# Starlark provides them itself

def expand_label(label):
    return "%s" % label if ":" in label else "%s:%s" % (label, label.split("/")[-1])
