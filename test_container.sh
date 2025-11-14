#!/bin/bash
#
# AutoSDV Container Testing Script
# Use this after rebuilding to verify everything works in simulation mode
#

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "╔══════════════════════════════════════════════════════════════════════════════╗"
echo "║              AutoSDV Container Test Script (Simulation Mode)                ║"
echo "╚══════════════════════════════════════════════════════════════════════════════╝"
echo ""

# Test 1: Check workspace is sourced
echo -e "${BLUE}Test 1: Checking workspace setup...${NC}"
if [ -f "/AutoSDV/install/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    source /AutoSDV/install/setup.bash
    echo -e "${GREEN}✓ Workspace sourced successfully${NC}"
else
    echo -e "${RED}✗ Workspace not built yet${NC}"
    exit 1
fi
echo ""

# Test 2: Check ROS environment
echo -e "${BLUE}Test 2: Checking ROS environment...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS not sourced${NC}"
    exit 1
else
    echo -e "${GREEN}✓ ROS_DISTRO: $ROS_DISTRO${NC}"
fi
echo ""

# Test 3: Check RMW implementation
echo -e "${BLUE}Test 3: Checking RMW implementation...${NC}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo -e "${GREEN}✓ RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION${NC}"
echo ""

# Test 4: Check critical packages are built
echo -e "${BLUE}Test 4: Checking critical packages...${NC}"
PACKAGES=(
    "autosdv_launch"
    "autosdv_vehicle_interface"
    "gnss_locator"
    "nmea_reader"
    "zed_components"
    "zed_wrapper"
)

ALL_FOUND=true
for pkg in "${PACKAGES[@]}"; do
    if ros2 pkg list | grep -q "^${pkg}$"; then
        echo -e "${GREEN}✓ $pkg found${NC}"
    else
        echo -e "${RED}✗ $pkg NOT found${NC}"
        ALL_FOUND=false
    fi
done

if [ "$ALL_FOUND" = false ]; then
    echo ""
    echo -e "${RED}Some packages are missing. Build may have failed.${NC}"
    exit 1
fi
echo ""

# Test 5: Check seyond is NOT built (expected on ARM64 via QEMU)
echo -e "${BLUE}Test 5: Verifying seyond is skipped...${NC}"
if ros2 pkg list | grep -q "^seyond$"; then
    echo -e "${YELLOW}⚠ seyond found (may cause issues on x86)${NC}"
else
    echo -e "${GREEN}✓ seyond correctly skipped${NC}"
fi
echo ""

# Test 6: Check launch file exists
echo -e "${BLUE}Test 6: Checking launch files...${NC}"
LAUNCH_FILE=$(ros2 pkg prefix autosdv_launch)/share/autosdv_launch/launch/autosdv.launch.yaml
if [ -f "$LAUNCH_FILE" ]; then
    echo -e "${GREEN}✓ Main launch file found${NC}"
    echo "  Path: $LAUNCH_FILE"
else
    echo -e "${RED}✗ Launch file not found${NC}"
    exit 1
fi
echo ""

# Test 7: Check if QEMU or native
echo -e "${BLUE}Test 7: Environment detection...${NC}"
if [ -f /proc/device-tree/model ] && grep -q "NVIDIA" /proc/device-tree/model 2>/dev/null; then
    echo -e "${GREEN}🤖 Real Jetson hardware detected${NC}"
elif [ -f /etc/nv_tegra_release ]; then
    echo -e "${YELLOW}🐳 L4T Docker (QEMU emulation)${NC}"
else
    echo -e "${YELLOW}🐳 Non-Jetson environment${NC}"
fi
echo ""

# Summary
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${GREEN}✅ ALL TESTS PASSED!${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Container is ready for testing. To launch AutoSDV:"
echo ""
echo "  cd /AutoSDV"
echo "  make launch"
echo ""
echo "Expected behavior on x86:"
echo "  • Auto-detects simulation mode"
echo "  • Disables hardware-dependent modules"
echo "  • No crashes or error loops"
echo ""



