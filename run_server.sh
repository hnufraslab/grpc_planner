#!/bin/bash

# Run script for gRPC Planner Service

set -e  # Exit on error

echo "=========================================="
echo "Starting gRPC Planner Service"
echo "=========================================="

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check if server executable exists
if [ ! -f "build/grpc_planner_server" ]; then
    echo "Error: Server executable not found!"
    echo "Please run ./build.sh first"
    exit 1
fi

# Set default environment variables if not set
export GRPC_SERVER_ADDRESS="${GRPC_SERVER_ADDRESS:-0.0.0.0:50051}"

echo ""
echo "Configuration:"
echo "  SERVER_ADDRESS: $GRPC_SERVER_ADDRESS"
echo ""
echo "Note: Point cloud will be provided via UpdatePointCloud interface"
echo "Note: No MuJoCo collision detection (collision checking disabled)"
echo ""

# Run the server
./build/grpc_planner_server
