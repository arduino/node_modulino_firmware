#!/bin/bash

# Make sure sketches/libraries/Modulino/src/ folder exists

if [ -d sketches/libraries/Modulino/src/ ]; then
CPU=-mcpu=cortex-m0 make clean
CPU=-mcpu=cortex-m0 make
echo const > sketches/libraries/Modulino/src/fw.h
cd build
xxd -i node_base.bin >> ../sketches/libraries/Modulino/src/fw.h
cd ..

CPU=-mcpu=cortex-m0 make clean
CPU=-mcpu=cortex-m0 EXTRA_CFLAGS=-DFORCE_LEDMATRIX_MODULINO make
echo const > sketches/libraries/Modulino/src/fw_ledmatrix.h
cd build
mv node_base.bin matrix_node_base.bin
xxd -i matrix_node_base.bin >> ../sketches/libraries/Modulino/src/fw_ledmatrix.h
cd ..

echo "Now you can create a commit in Modulino library"
else
echo "Please run git submodule init && git submodule update"
exit 1
fi
