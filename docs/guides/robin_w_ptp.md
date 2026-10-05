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

The step installs the `linuxptp`, `ethtool`, `netcat-openbsd`, and `chrony`
packages when needed. At boot, the PTP service waits for chrony to synchronize
the Jetson system clock and for NetworkManager to bring eno1 online before it
starts advertising time. This prevents the lidar from receiving the Unix epoch
as its initial timestamp.

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

Configure the Robin-W after the PTP services are active. AutoSDV includes the
repo-local `innovusion_lidar_util` helper because the original vendor utility
is x86_64-only. It sends the configuration command to the lidar's TCP port
8002; replace the address below with the lidar address on your network.

~~~bash
LIDAR_IP=172.168.1.10

./setup/files/bin/innovusion_lidar_util "$LIDAR_IP" get_config time ptp_en
./setup/files/bin/innovusion_lidar_util "$LIDAR_IP" set_config time ptp_en 1
./setup/files/bin/innovusion_lidar_util "$LIDAR_IP" set_config time ptp_automotive 0
./setup/files/bin/innovusion_lidar_util "$LIDAR_IP" get_config time ptp_en
./setup/files/bin/innovusion_lidar_util "$LIDAR_IP" get_config time ptp_automotive
~~~

The expected settings are:

- PTP enabled: ptp_en=1
- User-defined PTP mode: ptp_automotive=0

Then check that the lidar's own ptp4l config parses. Some units ship one
that does not; see Troubleshooting below:

~~~bash
./setup/files/bin/robin_w_ptp_config "$LIDAR_IP" check
~~~

This configuration is intentionally not gPTP. gPTP requires Layer-2 P2P
support throughout the network path and is a different Robin-W operating mode.

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

## Troubleshooting: the Robin-W never answers

Symptom: ptp4l.service on the Jetson is MASTER and sends Sync, Follow_Up and
Announce, but the lidar sends no Delay_Req, and its clock (`date` over SSH)
still reads its boot default, December 2022.

~~~bash
tshark -i eno1 -a duration:8 -f "udp port 319 or udp port 320"
~~~

A healthy lidar appears as `172.168.1.10 → 224.0.1.129 Delay_Req` about once a
second. If only 172.168.1.100 appears, check the lidar's own PTP config:

~~~bash
./setup/files/bin/robin_w_ptp_config "$LIDAR_IP" check
~~~

**Not every Robin-W has this problem.** `check` is read-only. It reports `OK`
or names the offending line, and only an affected unit needs `fix`.

### Cause

The lidar runs its own ptp4l (`/app/firmware/ptp/ptp4l`, started by
`ptp_start.sh` in a loop) against `/mnt/config_firmware/inno_internal_file_PTP`.
On an affected unit that file ends with:

~~~text
[eth0]
domainNumber 0
~~~

`domainNumber` is valid only in `[global]`. The bundled ptp4l fails with
`unknown option domainNumber at line 14 in eth0 section`. It exits at once and
the loop restarts it every 5 s, so the lidar never sends a Delay_Req. The lidar's
`phc2sys` reads the same file and dies the same way, which is why the lidar's
system clock never moves.

The lidar's web UI writes this line, and this is a vendor bug. Seen on firmware
`rwg-release-1563` / app `release-rwg-B3.0-2.0-lidar-app-rc1`. Pressing Save in
Time Sync Settings with source PTP sends `set_ptp_config PTP domainNumber N`.
`lidar_util.set_ptp_config` (`/app/python/ila/src/lidar_util.py`) removes any
existing line for the key and **appends** the new one at the end of the file,
which is under `[eth0]`. It does this even for the default domain 0, and even
when the line was already in `[global]`. **Do not press Save in the web UI's Time
Sync form**; configure with `innovusion_lidar_util` as above.

### Why editing the file over SSH does not stick

The config exists twice, `/mnt/config_firmware/` (mtdblock1) and
`/backup/config_firmware/` (mtdblock2), each with `.md5` and `.md5_2` sidecars.
lidar-app checks both copies against their checksums at boot and restores one
from the other (`restore backup file`, `restore origin file`), so an edit to the
`/mnt` copy alone is undone by the next reboot.

`robin_w_ptp_config` writes through lidar-app's `upload_internal_file PTP`
command on TCP 8002, the path the web UI itself uses. lidar-app verifies the
md5, writes through a temporary file, reads the result back from flash and
copies it, with both checksums, to `/backup`. A reboot then keeps the change.

### Fix

~~~bash
./setup/files/bin/robin_w_ptp_config "$LIDAR_IP" fix --dry-run   # show the result
./setup/files/bin/robin_w_ptp_config "$LIDAR_IP" fix             # upload it
~~~

`fix` downloads the unit's own file, moves global-only options out of `[eth0]`
into `[global]`, and keeps every other line as the unit has it. It saves the
original to `robin_w_PTP.<md5>.orig` in the current directory first, so
`put robin_w_PTP.<md5>.orig` restores it. `ptp_start.sh` picks up the new file
within about 5 s, without a reboot.

### If ptp4l is unstable after the fix

With the parse error gone, one unit's ptp4l reached SLAVE but did not hold it.
The vendor file sets `step_threshold 0.00002` (20 µs), so any offset above
20 µs steps the clock, and each step can raise the lidar's frame-sync fault
(FAULT 34, healing in about 1 s). It also has no `tx_timestamp_timeout`, and the
macb driver's TX timestamps exceed the default, which the log shows as
`timed out while polling for tx timestamp` followed by the port going FAULTY.
`fix --tune` sets `step_threshold 1.0` and `tx_timestamp_timeout 10`; the
lidar's own gPTP file uses `1.0` and `1000` for the same two options.

Measured on that unit (2026-10-05):

| | as shipped, 8 min | `--tune`, 10 min |
|---|---|---|
| ptp4l restarts | 9 | 0 |
| tx-timestamp timeouts | 4 | 0 |
| servo jumps | 24 | 4, all in the first lock |
| frame-sync faults | 2 or more | 0 |
| port state | LISTENING, UNCALIBRATED, SLAVE, FAULTY | SLAVE throughout |

After a reboot, the lidar kept the file, its `date` read the Jetson's time, and
the driver's `/iv_points` carried 2026 stamps, arriving 106–114 ms after the
stamp (one 100 ms frame plus transport). Two things remain:

- For about 2.5 minutes after power-on, ptp4l restarts a few times and the
  frame-sync fault comes and goes while the clock moves from 2022 to now.
- Most offsets are under 10 µs (typically tens to hundreds of ns), but about
  5–8% of samples spike above 100 µs, occasionally to about 1 ms. The source,
  Jetson or lidar timestamping, is not yet known.

To watch the lidar's servo, SSH in and run:

~~~bash
/app/firmware/ptp/pmc -u -b 0 -f /mnt/config_firmware/inno_internal_file_PTP \
    "GET TIME_STATUS_NP" "GET PORT_DATA_SET" | grep -E "master_offset|portState"
grep ptp4l /var/log/messages | tail
~~~

References:

- [Seyond LiDAR Time Synchronization AppNote](https://www.seyond.com/wp-content/uploads/2025/04/AppsNote-Seyond-LiDAR-Time-Synchronization_V1.0.0_EN_20250402.pdf)
- [linuxptp phc2sys documentation](https://github.com/richardcochran/linuxptp/blob/master/phc2sys.8)
