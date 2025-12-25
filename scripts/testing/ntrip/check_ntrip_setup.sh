#!/bin/bash
# NTRIP/RTK Setup Verification Script
# Verifies that all components for NTRIP/RTK are properly configured

set -e

echo "========================================="
echo "AutoSDV NTRIP/RTK Setup Verification"
echo "========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Source ROS environment
echo -n "Sourcing ROS environment... "
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
echo -e "${GREEN}OK${NC}"

# Check 1: NTRIP Client Package
echo ""
echo "1. Checking NTRIP Client Package..."
if ros2 pkg list | grep -q ntrip_client; then
    echo -e "   ${GREEN}✓${NC} ntrip_client package found"
    ros2 pkg executables ntrip_client | sed 's/^/     /'
else
    echo -e "   ${RED}✗${NC} ntrip_client package NOT found"
    echo "   Run: sudo apt install ros-humble-ntrip-client"
    exit 1
fi

# Check 2: Message Packages
echo ""
echo "2. Checking Message Packages..."
for pkg in rtcm_msgs nmea_msgs ublox_msgs; do
    if ros2 pkg list | grep -q "^$pkg$"; then
        echo -e "   ${GREEN}✓${NC} $pkg found"
    else
        echo -e "   ${RED}✗${NC} $pkg NOT found"
        exit 1
    fi
done

# Check 3: AutoSDV Sensor Kit Package
echo ""
echo "3. Checking AutoSDV Sensor Kit Configuration..."
if ros2 pkg list | grep -q autosdv_sensor_kit_launch; then
    echo -e "   ${GREEN}✓${NC} autosdv_sensor_kit_launch package found"

    # Check for NTRIP launch file
    NTRIP_LAUNCH=$(ros2 pkg prefix autosdv_sensor_kit_launch)/share/autosdv_sensor_kit_launch/launch/ntrip.launch.xml
    if [ -f "$NTRIP_LAUNCH" ] || [ -L "$NTRIP_LAUNCH" ]; then
        echo -e "   ${GREEN}✓${NC} ntrip.launch.xml found"
    else
        echo -e "   ${RED}✗${NC} ntrip.launch.xml NOT found"
        echo "   Expected: $NTRIP_LAUNCH"
        exit 1
    fi

    # Check for ZED-F9R config
    F9R_CONFIG=$(ros2 pkg prefix autosdv_sensor_kit_launch)/share/autosdv_sensor_kit_launch/config/zed_f9r_rover.yaml
    if [ -f "$F9R_CONFIG" ] || [ -L "$F9R_CONFIG" ]; then
        echo -e "   ${GREEN}✓${NC} zed_f9r_rover.yaml found"
    else
        echo -e "   ${RED}✗${NC} zed_f9r_rover.yaml NOT found"
        echo "   Expected: $F9R_CONFIG"
        exit 1
    fi
else
    echo -e "   ${RED}✗${NC} autosdv_sensor_kit_launch NOT found"
    echo "   Run: make build"
    exit 1
fi

# Check 4: Hardware (optional, won't fail)
echo ""
echo "4. Checking Hardware..."
if [ -e /dev/ublox-gps ]; then
    echo -e "   ${GREEN}✓${NC} u-blox GPS device found at /dev/ublox-gps"
    DEVICE_INFO=$(ls -l /dev/ublox-gps | awk '{print $NF}')
    echo "     → Points to: $DEVICE_INFO"
elif [ -e /dev/ttyACM0 ]; then
    echo -e "   ${YELLOW}⚠${NC} Found /dev/ttyACM0 but no /dev/ublox-gps symlink"
    echo "     → May need to install udev rules"
    echo "     → Run: sudo cp scripts/ublox-gps/99-ublox-gps.rules /etc/udev/rules.d/"
    echo "     →      sudo udevadm control --reload-rules && sudo udevadm trigger"
else
    echo -e "   ${YELLOW}⚠${NC} No u-blox GPS device detected"
    echo "     → Hardware not connected yet (this is OK for now)"
fi

# Check 5: Internet connectivity to e-GNSS
echo ""
echo "5. Checking NTRIP Server Connectivity..."
if timeout 3 bash -c "echo > /dev/tcp/210.241.63.193/81" 2>/dev/null; then
    echo -e "   ${GREEN}✓${NC} e-GNSS Taiwan VRS server reachable (210.241.63.193:81)"
else
    echo -e "   ${YELLOW}⚠${NC} Cannot reach e-GNSS server (may be offline or firewalled)"
    echo "     → Check internet connection"
    echo "     → Ensure TCP port 81 is not blocked"
fi

# Check 6: Documentation
echo ""
echo "6. Checking Documentation..."
if [ -f "docs/ntrip_rtk_testing.md" ]; then
    echo -e "   ${GREEN}✓${NC} Testing guide found: docs/ntrip_rtk_testing.md"
else
    echo -e "   ${YELLOW}⚠${NC} Testing guide not found"
fi

# Summary
echo ""
echo "========================================="
echo "Setup Verification Complete!"
echo "========================================="
echo ""
echo "Next Steps:"
echo "  1. Connect u-blox ZED-F9R GPS receiver via USB"
echo "  2. Verify device appears as /dev/ublox-gps"
echo "  3. Test NTRIP/RTK:"
echo "     make launch ARGS=\"gnss_receiver:=ublox use_ntrip:=true\""
echo ""
echo "For detailed testing procedures, see:"
echo "  docs/ntrip_rtk_testing.md"
echo ""
