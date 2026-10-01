# Seyond Robin-W PTP synchronization

AutoSDV configures the Jetson as the PTP grandmaster on eno1. The Jetson's
system clock is copied to the eno1 PTP hardware clock by phc2sys, and ptp4l
advertises that clock to the Robin-W using hardware-timestamped UDPv4 PTP with
end-to-end delay measurement.

## Install

The vehicle setup profile includes the PTP step:

~~~bash
./setup.sh --run --only robin-w-ptp --yes
~~~

The step requires linuxptp, ethtool, a carrier on eno1, and a PTP hardware
clock. If linuxptp is missing, install the distro package first:

~~~bash
sudo apt install linuxptp ethtool
~~~

The installer writes these repository-managed files to the system:

~~~text
/etc/linuxptp/autosdv-robin-w-ptp4l.conf
/etc/linuxptp/autosdv-robin-w-phc2sys.conf
/etc/systemd/system/ptp4l.service
/etc/systemd/system/phc2sys.service
~~~

It disables any ptp4l@eno1.service or phc2sys@eno1.service instances so two
linuxptp processes cannot claim the same PHC. It does not edit the
package-owned files in /lib/systemd/system.

## Robin-W setting

Configure the Robin-W with its vendor configuration utility or management
interface before testing:

- PTP enabled: ptp_en=1
- User-defined PTP mode: ptp_automotive=0

This configuration is intentionally not gPTP. gPTP requires Layer-2 P2P
support throughout the network path and is a different Robin-W operating mode.

Do not put lidar credentials in this repository. Use the Seyond-provided tool
or management interface for the sensor-side setting.

## Check the synchronization

First make sure the Jetson system clock is already correct. In this setup the
Jetson is the source of time; PTP cannot correct an incorrect system clock.

~~~bash
systemctl is-active ptp4l.service phc2sys.service
sudo journalctl -u ptp4l.service -u phc2sys.service -b --no-pager
sudo pmc -u -b 1 "GET TIME_STATUS_NP"
ethtool -T eno1
~~~

ptp4l should report the port in MASTER state, and phc2sys should remain active
while tracking CLOCK_REALTIME to the eno1 PHC. After the Robin-W is enabled for
PTP, compare its reported time and point timestamps with the Jetson clock.

References:

- [Seyond LiDAR Time Synchronization AppNote](https://www.seyond.com/wp-content/uploads/2025/04/AppsNote-Seyond-LiDAR-Time-Synchronization_V1.0.0_EN_20250402.pdf)
- [linuxptp phc2sys documentation](https://github.com/richardcochran/linuxptp/blob/master/phc2sys.8)
