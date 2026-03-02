#!/bin/bash

# Define paths
USERNAME=$(whoami)
QUARTUS_BIN="/home/$USERNAME/intelFPGA_lite/20.1/quartus/bin"
PROJECT_NAME="t6_maze_solver"
OUTPUT_DIR="output_files"
SOF_FILE="${OUTPUT_DIR}/${PROJECT_NAME}.sof"
JIC_FILE="${OUTPUT_DIR}/output_file.jic"
CDF_FILE="${OUTPUT_DIR}/jic_programming.cdf"
CABLE="USB-Blaster [1-3]"

echo "--------------------------------------------------------"
echo "Starting FPGA Program Sequence"
echo "--------------------------------------------------------"

# 4. Program the EPCS via JTAG using the CDF
echo "[4/4] Programming EPCS (Non-volatile JIC)..."
"$QUARTUS_BIN/quartus_pgm" -c "$CABLE" "$CDF_FILE"

if [ $? -ne 0 ]; then
    echo "Error: Programming failed."
    exit 1
fi

echo "--------------------------------------------------------"
echo "Build and Programming Complete!"
echo "--------------------------------------------------------"
