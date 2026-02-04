#!/bin/bash

# Build script for gRPC Planner Service (C++ only)

set -e  # Exit on error

echo "=========================================="
echo "Building gRPC Planner Service (C++ only)"
echo "=========================================="

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Step 1: Create build directory
echo ""
echo "Step 1: Creating build directory..."
mkdir -p build
cd build

# Step 2: Configure CMake
echo ""
echo "Step 2: Configuring CMake..."
cmake ..

if [ $? -eq 0 ]; then
    echo "✓ CMake configuration successful"
else
    echo "✗ CMake configuration failed"
    exit 1
fi

# Step 3: Build C++ server
echo ""
echo "Step 3: Building C++ server..."
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
