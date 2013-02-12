#!/bin/bash

# READLINE is nicer, but it dies when given lots of output
socat UDP4-RECV:6666,reuseaddr!!UDP4-SENDTO:robot:6668 STDIO
