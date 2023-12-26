#!/bin/bash

# Set the Fully Qualified Board Name (FQBN) for Arduino Nano 33 BLE
FQBN="arduino:mbed_nano:nano33ble"

# Set the sketch directory (replace 'my_sketch' with your actual sketch directory name)
SKETCH_DIR="$PWD/batfink"

# Set the build directory for efficiency
BUILD_DIR="$PWD/.build"

# Function to calculate elapsed time
calculate_time() {
    local end_time=$(date +%s)
    local elapsed_time=$((end_time - start_time))
    echo "Time taken: $elapsed_time seconds"
}

# Detecting Arduino Nano 33 BLE
echo "Detecting Arduino Nano 33 BLE..."
start_time=$(date +%s)
PORT=$(arduino-cli board list | grep 'Arduino Nano 33 BLE' | awk '{print $1}')
if [ -z "$PORT" ]; then
    echo "Arduino Nano 33 BLE not found. Please ensure it is connected."
    exit 1
fi
calculate_time
echo "Found Arduino Nano 33 BLE at port $PORT"

# Compile the sketch
echo "Compiling the sketch..."
start_time=$(date +%s)
arduino-cli compile --fqbn $FQBN --build-path $BUILD_DIR $SKETCH_DIR
calculate_time

# Upload the sketch
echo "Uploading the sketch to the board..."
start_time=$(date +%s)
arduino-cli upload -p $PORT --fqbn $FQBN --input-dir $BUILD_DIR
calculate_time

