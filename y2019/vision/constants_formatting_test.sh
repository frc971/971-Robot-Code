#!/bin/bash
# The constants file should not have changed in a way that will not be preserved
# when we rerun the codegen.
diff $1 $2
