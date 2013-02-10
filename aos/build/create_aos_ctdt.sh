#!/bin/bash

echo '#include "aos/crio/aos_ctdt.h"'
calls=''
for symbol in `cat - | awk '{ print $NF }' | grep '^aos_init_function_'`; do
	echo "void $symbol();"
	calls="$calls$symbol();\n"
done
echo 'void aos_call_init_functions() {'
echo -e $calls
echo '}'

