#!/bin/bash

# Build script for gRPC Planner Service

set -e  # Exit on error

echo "=========================================="
echo "Building gRPC Planner Service"
echo "=========================================="

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Step 1: Generate Python Protobuf files
echo ""
echo "Step 1: Generating Python Protobuf files..."
python3 -m grpc_tools.protoc \
    -I. \
    --python_out=. \
    --grpc_python_out=. \
    planner.proto

if [ $? -eq 0 ]; then
    echo "✓ Python Protobuf files generated successfully"
else
    echo "✗ Failed to generate Python Protobuf files"
    exit 1
fi

# Step 2: Create build directory
echo ""
echo "Step 2: Creating build directory..."
mkdir -p build
cd build

# Step 3: Configure CMake
echo ""
echo "Step 3: Configuring CMake..."
cmake ..

if [ $? -eq 0 ]; then
    echo "✓ CMake configuration successful"
else
    echo "✗ CMake configuration failed"
    exit 1
fi

# Step 4: Build C++ server
echo ""
echo "Step 4: Building C++ server..."
make -j$(nproc)

if [ $? -eq 0 ]; then
    echo "✓ Build successful"
else
    echo "✗ Build failed"
    exit 1
fi

echo ""
echo "=========================================="
echo "Build completed successfully!"
echo "=========================================="
echo ""
echo "Executable: $SCRIPT_DIR/build/grpc_planner_server"
echo ""
echo "To run the server:"
echo "  ./run_server.sh"
echo ""
echo "To run the Python client:"
echo "  python3 grpc_client.py"
echo ""
