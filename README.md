# Husky autonomous patrol — ROS 2 / Gazebo

The existing `husky_patrol` model now runs the accepted indoor/outdoor route using wheel physics, measured world pose, a LiDAR collision guard, route-wide anomaly detection and saved robot-camera evidence. Its original chassis, wheels, bumpers and meshes are retained. The earlier `patrol_rover` model remains in the workspace as a reference; the default launch uses Husky.

## Run

Stop the previous simulation with **Ctrl+C**, then:

```bash
cd ~/werob_layout
./werob.sh
```

The script builds the package, sources ROS 2 Jazzy and opens Gazebo. The launch terminal shows patrol, reference, turn and anomaly messages. Saved diagnostic logs in `archive/validation/` are recordings, not the current console. `source werob.sh` only prepares another terminal.

The default renderer is Ogre with Mesa llvmpipe software rendering. The local Intel/Ogre driver previously returned 720 identical minimum-range readings; saturated scans now hold the robot and print `LIDAR RENDERING ERROR`. The launch also removes incompatible Snap editor GTK/GIO paths.

## Indoor and outdoor demonstrations

Choose one scenario per launch:

```bash
./werob.sh obstacle_mode:=beside     # default: pallet beside the indoor route
./werob.sh obstacle_mode:=outdoor    # pallet beside the outdoor route
./werob.sh obstacle_mode:=mixed      # one outdoor and one indoor pallet
./werob.sh obstacle_mode:=blocking   # indoor pallet obstructs the route
./werob.sh obstacle_mode:=none       # clear baseline; add your own object later
```

1. **First lap:** Husky records LiDAR scans and the space those rays observed as clear. It completes the entire route before freezing the reference (`REFERENCE_FROZEN`).
2. **After that lap:** the selected demonstration objects are inserted automatically. For manual tests, add or move an object **after** the reference has frozen.
3. **Later laps:** persistent new surfaces in previously observed free space generate `ANOMALY DETECTED`, indoors or outdoors. Several distinct locations can generate incidents in the same lap.
4. A route obstruction independently generates `COLLISION STOP`. Husky holds the current waypoint until clearance returns; it does not reroute or skip waypoints.

An object already present during the first lap belongs to the reference. Previously unseen or occluded space is not assumed to be clear. The detector does not receive spawn coordinates or use insertion timing to invent an alarm.

## Camera pictures and incident records

Each incident writes a JSON record and, when a camera frame is available within 0.5 simulation seconds, the actual camera image:

```text
incidents/<session>/<event-id>.json
incidents/<session>/<event-id>.png
```

The console prints `CAMERA IMAGE SAVED` with the path. A different directory can be selected:

```bash
./werob.sh obstacle_mode:=mixed evidence_dir:=$PWD/incidents
```

The record includes the object surface's map coordinates, robot pose, incident time, image time, frame offset, LiDAR comparison and camera path. These provide a human operator with an image and location to review later.

`camera_capture_status` distinguishes `projected_object_view` from `context_only`. The camera faces forward, so a side/rear LiDAR detection may be outside the picture. Projection uses the nominal level-robot camera geometry; it is an estimate, not visual recognition or proof of visibility. Missing/stale frames are recorded explicitly. The robot does not automatically turn toward an incident, and no remote guidance interface has been added.

**Detection currently uses LiDAR.** Camera pixels are saved as evidence; camera AI, object classification and privacy masking are not implemented. Images are local files and are not uploaded.

## Husky motion and sensors

Navigation uses Gazebo's measured `/odom` pose rather than wheel-integrated `/wheel_odom`. Skid-steering wheel motion can differ from actual chassis rotation. Husky rotates until the measured heading error is below 0.045 radians (about 2.6°) before advancing. `turn_completed` reports the measured turn and remaining heading error; quarter turns are approximately 90°, and the indoor return is a U-turn.

The four wheel joints retain their original 0.5708 m centre separation and 0.1651 m collision radius. Lateral slip is enabled for skid steering. The collision guard accounts for the bumpers with a conservative 1.12 × 0.70 m footprint.

The upgraded LiDAR has 720 horizontal samples, a 25 m range and a scan plane approximately 0.53 m above ground. The camera captures 960 × 720 pixels with a 1.2 radian horizontal field of view. Its nominal height is approximately 0.65 m. Contacts from the chassis and bumpers are bridged on `/patrol/contacts`.

## Route and detection settings

```text
dock (0,0) → yard_north (7.5,0) → gate (12,0)
→ yard_south (12,-5) → corridor_entry (5,-5)
→ bottleneck_in (1,-5) → indoor_checkpoint (-5.5,-5)
→ bottleneck_out (1,-5) → corridor_exit (5,-5)
→ outdoor_return (5,0) → dock (0,0)
```

The building has a rear wall and a framed 1.2 m wide, 2.05 m high doorway centred on the route. Its wall connections preserve the accepted path.

`src/patrol_simulation/reference/monitored_zones.yaml` configures thresholds and region labels. **Region labels do not restrict detection.** The detector requires four adjacent changed rays, three consecutive observations, a range reduction greater than 0.5 m, and a location observed as free on the reference lap. A 0.15 m grid and a two-cell uncertainty band around baseline surfaces reduce false alarms near walls. Repeat observations within 0.9 m of an already reported location are suppressed for the run; a sufficiently displaced location can generate another incident. This is location association, not semantic object identity tracking.

The single horizontal LiDAR plane cannot detect every low or elevated object. Small changes close to reference surfaces, occlusion, insufficient returns and poor reference coverage can prevent an anomaly report. The immediate collision guard operates independently of the reference and region labels.

## Topics and validation

In another terminal:

```bash
cd ~/werob_layout
source werob.sh
ros2 topic echo /patrol/events --qos-durability transient_local --field data
```

Other useful topics are `/patrol/status`, `/patrol/reference_observation`, `/odom`, `/scan`, `/camera/image` and `/patrol/contacts`.

```bash
PYTHONPATH=src/patrol_simulation python3 -m pytest -q src/patrol_simulation/test
bash scripts/validate_live.sh --mode mixed --gui --label husky_mixed
bash scripts/validate_live.sh --mode none --label husky_clear
bash scripts/validate_live.sh --mode blocking --label husky_blocking
```

The validator isolates its ROS domain and Gazebo partition. Selected events, trajectories, camera evidence and results are kept in `validation/`; raw scan dumps and console logs go into the ignored `archive/validation/` folder. See [VALIDATION.md](VALIDATION.md) for current results and [the evidence index](validation/README.md) for the files included in the submission.

Prerequisites: ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`, Python YAML, Pillow and `colcon`. Localisation is simulator ground truth, not SLAM. Production camera/thermal analytics, incident review services and reporting workflows described in [CASE_STUDY_RESPONSE.md](CASE_STUDY_RESPONSE.md) remain outside this simulation implementation.


## Submission contents

- `src/patrol_simulation/`: active package, Husky and rover models, pallet, world, configuration and tests.
- `scripts/` and `werob.sh`: launching, validation, route plotting and GUI capture.
- [CASE_STUDY_RESPONSE.md](CASE_STUDY_RESPONSE.md): Tasks 1, 3 and 4, plus the prototype summary.
- [event_examples.json](event_examples.json): illustrative proposed control-centre payloads for Task 3.
- [VALIDATION.md](VALIDATION.md) and [validation/](validation/README.md): selected measured Husky evidence for Task 2.

`.gitignore` excludes the local archive, build/install directories, routine logs, runtime incidents, editor databases and Python caches. It includes only the selected validation evidence. The original environment, unused accessories and old tests remain recoverable locally in `archive/`; that folder is not part of the submission. Build output is regenerated by `./werob.sh`.

## Approximately three-minute recording

The video has not yet been recorded. Run `./werob.sh obstacle_mode:=mixed` and keep Gazebo and the launch terminal visible. Aim for roughly 20 seconds introducing the Husky and route, 50 seconds showing the clean first lap and `REFERENCE_FROZEN`, 70 seconds showing outdoor and indoor `ANOMALY DETECTED` events and the saved camera pictures, and 40 seconds showing lap completion and the validation results. Rendering may take longer than three real-time minutes; trim uneventful straight segments and label any sped-up footage. Keep each displayed alarm and image tied to the same recorded run. A separate blocking demonstration is available with `obstacle_mode:=blocking`.

Attach the final recording or add a link to it before submitting; no recording file or link is included yet.
