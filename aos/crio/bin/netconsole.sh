#!/bin/bash

socat UDP4-RECV:6666,reuseaddr!!UDP4-SENDTO:robot:6668 READLINE
