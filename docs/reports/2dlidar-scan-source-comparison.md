# Choosing the 2-D scan source: slab, one ring, or a ring group

Three ways to produce MCL's 2-D scan from a 3-D LiDAR, measured against NDT
ground truth on the Autoware sample site. Same bag, same map, same five seeds,
same oracle initialisation, ~2239 paired poses per seed. Only the scan source
differs.

| source | mean, median | mean, range | gate 1.0 m | p95 | mean \|yaw\| | pairs |
|---|---|---|---|---|---|---|
| slab, 0.30 m band | 0.821 m | 0.807–0.879 | 5/5 | 2.148 m | 0.0339 rad | 2238–2239 |
| 1 ring (71) | 0.992 m | 0.761–1.078 | **3/5** | 2.081 m | 0.0321 rad | **1813**–2239 |
| **3 rings (70–72)** | **0.789 m** | **0.779–0.816** | **5/5** | **2.075 m** | **0.0159 rad** | 2239 |

**Three adjacent rings is the best of the three on every metric**, and it is also
the most faithful to a real 2-D LiDAR of the two that pass the gate.

## Why a group beats a single ring

A single ring is geometrically perfect — one elevation, so zero vertical extent
at any range, exactly what a 2-D LiDAR is. Its problem is density: one channel of
a 128-ring spinner is sparse, so each filter update carries little evidence. The
symptom is not a bias but *variability*: the mean spread across seeds is 0.317 m
for one ring against 0.037 m for three, an eightfold difference, and two of five
seeds miss the gate.

Adjacent VLS128 channels sit 0.11° apart, measured, so a small group stays close
to planar:

| group | span | z extent @10 m | @30 m | @60 m |
|---|---|---|---|---|
| real 2-D LiDAR | 0.00° | 0.00 m | 0.00 m | 0.00 m |
| 1 ring | 0.00° | 0.00 | 0.00 | 0.00 |
| 3 rings 70–72 | 0.22° | 0.04 | 0.12 | 0.23 |
| 5 rings 69–73 | 0.44° | 0.08 | 0.23 | 0.46 |
| slab | fixed | 0.30 | 0.30 | 0.30 |

Three rings is therefore *tighter than the slab* beyond about 30 m while carrying
roughly three times a single ring's returns. That combination — nearly planar and
reasonably dense — is why it wins.

## The unexplained result: heading

Mean |yaw| error **halves** with three rings: 0.0159 rad against 0.0339 for the
slab and 0.0321 for one ring. Translational accuracy improving with density was
predicted; this was not.

A plausible mechanism is that the slab mixes returns from several elevations
whose apparent bearing differs slightly, smearing the angular structure the
filter uses to resolve heading, while a tight group preserves it. **These runs do
not establish that.** It would be tested by comparing yaw error against group
width directly — 1, 3, 5, 9 rings — which is cheap and has not been done.

## Also worth noting

The single-ring run's seed 1 produced **1813 pairs against 2239** elsewhere, a
19% loss that is unexplained. It makes that seed's 0.889 m the least trustworthy
figure in the table, and it did not recur in the slab or 3-ring runs, both of
which produced full pair counts on every seed.

## Recommendation

Use a **small ring group** rather than a single ring for a 3-D LiDAR standing in
for a 2-D one. For the VLS128 in the sample bag that is channels 70–72:

```bash
ros2 launch autosdv_sensor_kit_launch scan_from_ring.launch.xml \
    input_topic:=/sensing/lidar/top/pointcloud_raw_ex \
    ring_min:=70 ring_max:=72
```

The group is sensor-specific and must be measured, not copied: 0.11° spacing is a
property of this VLS128. A VLP-32C's coarser layout would make three channels a
much wider wedge, so the same group width would be less faithful there. Use
`scripts/sensor/inspect_rings.py` to find the horizontal channel and read off its
neighbours' elevations.

## Reproducing

```bash
# slab
SCAN_MODE=slab OUT_DIR=.../phase6-scanarch scripts/2dlidar/run-mcl-e2e-matrix.sh
# one ring
SCAN_MODE=ring SCAN_RING=71 OUT_DIR=.../phase6-ring scripts/2dlidar/run-mcl-e2e-matrix.sh
# three rings
SCAN_MODE=ring SCAN_RING_MIN=70 SCAN_RING_MAX=72 OUT_DIR=.../phase6-ring3 \
    scripts/2dlidar/run-mcl-e2e-matrix.sh

# fidelity table
python3 tmp/ring_span.py
```
