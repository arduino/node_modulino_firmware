#!/bin/bash

# Make sure sketches/libraries/Modulino/src/ folder exists

if [ ! -d sketches/libraries/Modulino/src/ ]; then
echo "Please run git submodule init && git submodule update"
exit 1
fi

make cleanall # Clean previous builds including binary files

CPU=-mcpu=cortex-m0 make
echo const > sketches/libraries/Modulino/src/fw.h
xxd -i build/bin/node_base.bin >> sketches/libraries/Modulino/src/fw.h

make clean
CPU=-mcpu=cortex-m0 TARGET=matrix_node_base EXTRA_CFLAGS=-DFORCE_LEDMATRIX_MODULINO make
echo const > sketches/libraries/Modulino/src/fw_ledmatrix.h
xxd -i build/bin/matrix_node_base.bin >> sketches/libraries/Modulino/src/fw_ledmatrix.h

echo "Now you can create a commit in Modulino library"
