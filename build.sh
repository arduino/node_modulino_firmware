#!/bin/bash

# Make sure sketches/libraries/Modulino/src/ folder exists

if [ ! -d sketches/libraries/Modulino/src/ ]; then
echo "Please run git submodule init && git submodule update"
exit 1
fi

# Check if debug mode is enabled (-d)
if [ "$1" == "-d" ]; then
    echo "Debug mode enabled"
    DEBUG=1
else
    DEBUG=0
fi

make cleanall # Clean previous builds including binary files

CPU=-mcpu=cortex-m0 DEBUG_MODE=$DEBUG make
echo const > sketches/libraries/Modulino/src/fw.h
xxd -i -n node_base.bin build/bin/node_base.bin >> sketches/libraries/Modulino/src/fw.h

make clean
CPU=-mcpu=cortex-m0 DEBUG_MODE=$DEBUG TARGET=matrix_node_base EXTRA_CFLAGS=-DFORCE_LEDMATRIX_MODULINO make
echo const > sketches/libraries/Modulino/src/fw_ledmatrix.h
xxd -i -n matrix_node_base.bin build/bin/matrix_node_base.bin >> sketches/libraries/Modulino/src/fw_ledmatrix.h

make clean
CPU=-mcpu=cortex-m0 DEBUG_MODE=$DEBUG TARGET=motors_node_base EXTRA_CFLAGS=-DMODULINO_MOTORS_BUILD make
echo const > sketches/libraries/Modulino/src/fw_motors.h
xxd -i -n motors_node_base.bin build/bin/motors_node_base.bin >> sketches/libraries/Modulino/src/fw_motors.h

echo "✅ Build successful. Now you can create a commit in Modulino library"
