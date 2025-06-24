#!/usr/bin/env bash
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

export AUTOWARE_HOME=/opt/autoware
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

unset ROS_LOCALHOST_ONLY

# Check network parameters and enable CycloneDDS config accordingly.
if (ip link show eth0 | grep -q "MULTICAST") 2>/dev/null; then
    multicast=true
else
    multicast=false
fi

if [ \
     "$(sysctl -n net.core.rmem_max 2>/dev/null)" = 2147483647 \
     -a \
     "$(sysctl -n net.ipv4.ipfrag_time 2>/dev/null)" = 3 \
     -a \
     "$(sysctl -n net.ipv4.ipfrag_high_thresh 2>/dev/null)" = 134217728 \
  ]
then
    network_ok=true
else
    network_ok=false
fi

if $multicast && $network_ok; then
    export CYCLONEDDS_URI="file://$script_dir/cyclonedds.xml"
else
    echo 'warning: The system is not configured properly.'
    echo 'warning: CycloneDDS configuration is not applied.'
    echo 'warning: Did you run autoware-setup?'
fi

source /opt/ros/humble/setup.$(basename $(echo $SHELL))
