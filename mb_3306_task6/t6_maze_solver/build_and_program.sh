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
echo "Starting FPGA Build and Program Sequence"
echo "--------------------------------------------------------"

# 1. Compile the Project
echo "[1/4] Compiling Project: $PROJECT_NAME..."
"$QUARTUS_BIN/quartus_sh" --flow compile $PROJECT_NAME

if [ $? -ne 0 ]; then
    echo "Error: Compilation failed."
    exit 1
fi

# 2. Generate JIC File
echo "[2/4] Generating JIC File..."
# Check if SOF exists
if [ ! -f "$SOF_FILE" ]; then
    echo "Error: SOF file not found at $SOF_FILE"
    exit 1
fi

# Convert SOF to JIC for EPCS64 attached to EP4CE22
"$QUARTUS_BIN/quartus_cpf" -c -d EPCS64 -s EP4CE22 "$SOF_FILE" "$JIC_FILE"

if [ $? -ne 0 ]; then
    echo "Error: JIC generation failed."
    exit 1
fi

# 3. Generate CDF File for JIC Programming
echo "[3/4] Generating Programming Helper (CDF)..."
cat > "$CDF_FILE" <<EOL
/* Quartus Prime Version 20.1.0 Build 711 06/05/2020 SJ Lite Edition */
JedecChain;
	FileRevision(JESD32A);
	DefaultMfr(6E);

	P ActionCode(Cfg)
		Device PartName(EP4CE22F17) Path("$OUTPUT_DIR/") File("output_file.jic") MfrSpec(OpMask(1) SEC_Device(EPCS64) Child_OpMask(1 1));

ChainEnd;

AlteraBegin;
	ChainType(JTAG);
AlteraEnd;
EOL

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
