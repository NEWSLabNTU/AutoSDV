# Simulator Vehicle Modeling — CARLA vs Isaac Sim

Research notes on bringing a realistic AutoSDV vehicle into a simulator
without an in-house 3D mesh artist. Focus: **time-to-first-drive** and
reuse of existing AutoSDV assets (URDF, sensor params).

Date surveyed: 2026-04-20
Target Isaac Sim version: 5.0+ (NuRec / 3DGUT support)

---

## TL;DR

- **Isaac Sim** is the fast path for AutoSDV. URDFs import directly; the
  PhysX Vehicle Wizard generates a drivable car in minutes.
- **CARLA** requires Unreal Engine skeletal-mesh rigging. Weeks, not days.
- **Smartphone photogrammetry (NVIDIA NuRec + 3DGUT)** is production-usable
  for *visual* fidelity but does **not** provide physics or LiDAR returns.
  Use it as a photoreal skin on top of a Wizard-generated physics body.

## CARLA vs Isaac Sim — Workflow Comparison

| Dimension                | CARLA                                         | Isaac Sim                                     |
| ------------------------ | --------------------------------------------- | --------------------------------------------- |
| Asset format             | FBX skeletal mesh bound to `VehicleSkeleton`  | URDF → USD (native importer)                  |
| Toolchain                | Build-from-source + Unreal Engine Editor      | Isaac Sim only                                |
| Mesh rigging required    | Yes (chassis + 4 wheels to base skeleton)     | No (wizard generates primitives)              |
| Reuse AutoSDV URDF       | No direct path                                | One-click import                              |
| Tri budget               | 50k–100k                                      | No hard limit                                 |
| Post-import work         | Blueprints, physical asset, materials, glass  | Joint drive gains, sensor graph               |
| Time to first drive      | Days to weeks                                 | Minutes to hours                              |
| Photoreal rendering      | Strong (Unreal)                               | Good; photoreal via NuRec / 3DGUT scans       |
| ROS 2 bridge             | Via `carla-ros-bridge`                        | Native extension                              |
| AD scenario library      | Mature (traffic agents, weather)              | Smaller, growing                              |

**Recommendation:** Isaac Sim for AutoSDV. CARLA only if photoreal urban
traffic scenarios are the top priority and Unreal expertise exists.

## Isaac Sim — Vehicle Creation Paths

### Path 1: PhysX Vehicle Wizard (fastest, no mesh)
- Built-in GUI generates a 4-wheel vehicle: chassis + suspension + tires +
  engine model.
- Exports `.usd`. Drivable immediately.
- Wheelbase/track can be set to match
  `src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/actuator.yaml`.

### Path 2: URDF Import
- `File > Import` on an existing URDF; converts to USD.
- For mobile base, set drive target type to **Velocity** and enable
  **movable base**.
- Works if a URDF already exists in `autosdv_vehicle_launch`.

### Path 3: Hybrid (recommended for realism)
1. Use the Wizard for physics (correct wheelbase, collision, tire model).
2. Parent a scanned visual mesh under the wizard chassis as a visual-only
   child; keep wizard collision shapes.
3. Wire ROS 2 via the Wizard Vehicle + ROS 2 Controller pattern so
   Autoware's `/control/command/control_cmd` drives it.

## Smartphone Photogrammetry (Option A) — Maturity Assessment

Official NVIDIA pipeline as of late 2025. Status: **production-usable for
visuals, not turnkey for physics.**

### Pipeline
```
phone video → COLMAP → 3DGUT reconstruction → USDZ → File > Import
```

### What's Mature
- NVIDIA-published tutorial (Nov 2025) with named tooling (NuRec, 3DGUT).
- Reproducible; no custom scripts beyond standard flags.
- Isaac Sim 5.0+ renders USDZ splats natively via Fabric Scene Delegate.
- Hardware bar is low: modern phone, ~60% frame overlap, consistent
  lighting.
- Supporting ecosystem: SAGE-3D InteriorGS dataset, GaussGym, RE³SIM,
  RoboGSim.

### What's NOT Mature
- **Splats have no physics.** 3DGS is a rendering primitive only — no
  collision, no rigid body dynamics.
- **Splats do not produce LiDAR / depth returns.** RGB camera simulation
  only.
- **Overlapping splats do not blend correctly** in camera view; flat-card
  artifacts visible.
- **Scale/metric accuracy imperfect.** Open forum issue on measurement
  drift vs. real dimensions.
- Workaround is mandatory: ship a collision mesh or bounding proxy
  alongside the splat (RE³SIM pattern).

### Implication for AutoSDV
- Camera-only sim: Option A works today.
- Driving dynamics + LiDAR sim: requires PhysX Vehicle Wizard body + a
  collision proxy beneath the splat.
- Sim-to-real research (RE³SIM-style): this is the canonical use case.

**Verdict:** adopt now with eyes open. Treat smartphone scan as a
photoreal skin, not a physical vehicle.

## Alternatives to Photogrammetry (no mesh artist)

| Tool                                | Notes                                           |
| ----------------------------------- | ----------------------------------------------- |
| iPhone Pro LiDAR + Polycam          | Handheld scan → OBJ/FBX; Isaac Assimp converts  |
| Scaniverse                          | Free iOS app, splat + mesh output               |
| Revopoint / Einstar structured light| Higher fidelity, dedicated scanner              |
| 3DGUT via COLMAP                    | Best visual fidelity; no physics (see Option A) |

## Suggested AutoSDV Path

1. Scan the physical AutoSDV with an iPhone + Polycam (~1 hr).
2. Run PhysX Vehicle Wizard in Isaac Sim; set wheelbase/track from
   `actuator.yaml`.
3. Parent scanned mesh under the wizard chassis as visual-only child.
4. Configure ROS 2 bridge so Autoware control commands drive the wizard
   vehicle.
5. If LiDAR sim matters, add a simplified collision proxy matching the
   vehicle envelope.

## References

- [NVIDIA Blog: Reconstruct a Scene in Isaac Sim Using Only a Smartphone](https://developer.nvidia.com/blog/reconstruct-a-scene-in-nvidia-isaac-sim-using-only-a-smartphone/)
- [NVIDIA NuRec: Reconstruct Scenes from Mono Camera Data](https://docs.nvidia.com/nurec/robotics/neural_reconstruction_mono.html)
- [GitHub: nv-tlabs/3dgrut](https://github.com/nv-tlabs/3dgrut)
- [NVIDIA Forum: 3DGS Measurement Issues in Omniverse](https://forums.developer.nvidia.com/t/3d-gaussian-splatting-3dgs-measurement-issues-in-omniverse/346372)
- [RE³SIM: High-Fidelity Sim Data via 3DGS + PhysX](https://xshenhan.github.io/Re3Sim/)
- [RoboGSim: Real2Sim2Real Gaussian Splatting Simulator](https://robogsim.github.io/)
- [Isaac Sim: URDF Importer Extension (6.0)](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html)
- [Isaac Sim: Import URDF Tutorial](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/import_urdf.html)
- [Forum: Wizard Vehicle and ROS 2 Controller](https://forums.developer.nvidia.com/t/wizard-vehicle-and-ros2-controller/254812)
- [Forum: Adding Vehicles with PhysX Vehicle Dynamics (Python)](https://forums.developer.nvidia.com/t/adding-vehicles-in-isaac-sim-using-python-utilising-physx-vehicle-dynamics/250621)
- [CARLA: Add a New Vehicle](https://carla.readthedocs.io/en/latest/tuto_A_add_vehicle/)
- [CARLA: Content Authoring — Vehicles](https://carla.readthedocs.io/en/latest/tuto_content_authoring_vehicles/)
