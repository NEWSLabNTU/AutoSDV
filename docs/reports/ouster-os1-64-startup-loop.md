# Ouster OS1-64 never leaves `INITIALIZING`

Date: 2026-10-05. Bring-up of a used Ouster OS1-64 on the Jetson's `eno1`.
Status: **open** — narrowed to the spin-up / firing path; power under load is
the next thing to measure.

## The sensor

| field | value |
|---|---|
| `prod_line` | `OS-1-64` (Gen1, 2019 serial) |
| `prod_sn` | `991925000321` |
| `prod_pn` | `840-101855-02` |
| `base_pn` / `base_sn` | `000-101323-03` / `101837000676` |
| `proto_rev` | `v1.1.1` |
| MAC | `bc:0f:a7:00:05:a2` (`fe80::be0f:a7ff:fe00:5a2`) |
| mDNS | `os1-991925000321.local` (Gen1 uses `os1-`, not `os-`) |
| firmware | was `v2.1.1`, reflashed to `ousteros-image-prod-aries-v2.4.0+20220921174636` |

## Networking

The sensor is a DHCP **client**, not a server. With no DHCP server on the link
it falls back to IPv4 link-local (`169.254.117.249/16` at the time). The
NetworkManager profile on `eno1` (`Wired connection 1`) was originally DHCP,
so neither side got an address; it is now set to `ipv4.method link-local`, and
the sensor is reached by its mDNS name, which is the only stable address in
that setup.

Discovery that needs no IPv4 and no root:

```bash
ping -6 -c 3 ff02::1%eno1        # all-nodes; Ouster MACs start bc:0f:a7
ip -6 neigh show dev eno1
curl -g 'http://[fe80::be0f:a7ff:fe00:5a2%25eno1]/api/v1/system/network/ipv4'
```

Browsers do not accept a `%zone` in a URL, so the web UI needs an IPv4 or mDNS
address.

Config still to fix once it runs: `udp_dest` is `169.254.177.121`, an old host
address. Set it to `@auto` (or the Jetson's address) and `save_config_params`.

API notes for this firmware: `/api/v1/sensor/metadata/sensor_info` and
`/api/v1/sensor/alerts` return 404 on 2.1.1; use the command interface,
`/api/v1/sensor/cmd/get_sensor_info` and `/api/v1/sensor/cmd/get_alerts`.

## The fault

`status` stays `INITIALIZING`. `get_alerts` shows an endless loop of one alert,
with an empty `msg_verbose`:

```
0x0100002a STARTUP WARNING
"Unit has experienced an internal warning during startup and is restarting."
```

raised, then "Cleared by reinitialization." ~100 ms later. `realtime` is
nanoseconds since boot.

| firmware | first alert | period |
|---|---|---|
| v2.1.1 | (looping from boot; cursor ~180 by 30 min) | 10.4 s |
| v2.4.0 | 15.0 s after boot | 12.5 s |

Same failure on both versions: **not firmware.**

## The test that narrowed it

```bash
S=http://os1-991925000321.local/api/v1/sensor/cmd
curl -s "$S/set_config_param?args=operating_mode%20STANDBY"
curl -s "$S/reinitialize"
```

Result: `status` reached `STANDBY`, and the alert cursor stopped at 35 across
repeated checks. The processor, network, firmware and core boards are healthy;
the fault appears only when the unit spins up and fires. The sensor was left in
STANDBY.

## Next

1. Meter on the interface-box input, switch back to `NORMAL`, watch a few
   ~12 s cycles. A sag well below 24 V each cycle confirms power.
2. Swap to a 24 V, ≥3 A supply on the shortest cable; check the interface-box
   connectors.
3. Observe the motor in `NORMAL`: spins up then stops each cycle / never spins
   / spins steadily while still looping. The last two, on good power, mean an
   Ouster repair (motor/driver, or laser/receiver path).
4. For Ouster support: serial, part number, both alert logs, and the web UI
   diagnostics dump.

Restore normal operation with `operating_mode%20NORMAL` + `reinitialize`.
