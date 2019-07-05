#!/bin/bash

source /home/lxl/Toolchain/gcc-arm-6.2/env6.2.txt

make clean
sleep 0.5
make
sleep 0.5
rm /home/lxl/test/globalmem.ko
cp -f /home/lxl/bit/ch6/globalmem.ko /home/lxl/test/globalmem.ko