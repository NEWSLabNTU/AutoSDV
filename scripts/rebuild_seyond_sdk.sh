#!/bin/bash
# Script to rebuild seyond SDK from source for ARM64
# This is needed because git submodule contains x86-64 precompiled libraries

set -e

SDK_PATH="/AutoSDV/src/sensor_component/external/seyond_ros_driver/src/seyond_lidar_ros/src/seyond_sdk"

if [ ! -d "$SDK_PATH" ]; then
    echo "Seyond SDK not found at $SDK_PATH, skipping rebuild"
    exit 0
fi

echo "════════════════════════════════════════════════════════════════"
echo "Rebuilding seyond SDK from source for ARM64"
echo "════════════════════════════════════════════════════════════════"

cd "$SDK_PATH"

# Check if libraries exist and what architecture they are
if [ -f "lib/libinnolidarsdkclient.a" ]; then
    echo "Current library info:"
    file lib/libinnolidarsdkclient.a | head -1
    
    # For .a archives, extract an object file to check architecture
    ARCH_CHECK=$(ar p lib/libinnolidarsdkclient.a | head -c 20 | file - | head -1)
    echo "Archive contents: $ARCH_CHECK"
    
    # Check if it's x86-64 (EM: 62 means x86-64, EM: 183 means ARM64)
    if echo "$ARCH_CHECK" | grep -q "x86-64"; then
        echo "⚠️  Detected x86-64 precompiled libraries, rebuilding for ARM64..."
        echo "Removing old libraries..."
        rm -f lib/*.a lib/*.so*
    elif echo "$ARCH_CHECK" | grep -q "aarch64\|ARM aarch64"; then
        echo "✓ ARM64 libraries already present, skipping rebuild"
        exit 0
    else
        echo "⚠️  Unknown architecture, rebuilding to be safe..."
        rm -f lib/*.a lib/*.so*
    fi
else
    echo "No existing libraries found, building from source..."
fi

# Build the SDK from source
echo "Compiling seyond SDK for ARM64..."
cd build
chmod +x build_unix.sh
./build_unix.sh

# Verify the result
cd ..
if [ -f "lib/libinnolidarsdkclient.a" ]; then
    echo "Build completed. Library info:"
    file lib/libinnolidarsdkclient.a
    
    # Check the object files inside the archive
    VERIFY_ARCH=$(ar p lib/libinnolidarsdkclient.a | head -c 20 | file - | head -1)
    echo "Archive contents: $VERIFY_ARCH"
    
    if echo "$VERIFY_ARCH" | grep -q "aarch64\|ARM aarch64"; then
        echo "✓ Successfully built ARM64 libraries"
    else
        echo "❌ ERROR: Built libraries are not ARM64!"
        echo "Got: $VERIFY_ARCH"
        exit 1
    fi
else
    echo "❌ ERROR: Build failed - library not found"
    exit 1
fi

echo "════════════════════════════════════════════════════════════════"
echo "Seyond SDK rebuild complete"
echo "════════════════════════════════════════════════════════════════"
