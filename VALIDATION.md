# Patrol collision and anomaly validation

## Current Husky validation — 10 September 2026

The default launch now uses the existing `husky_patrol` meshes, chassis and wheels, with raised sensors, mounting brackets, measured world odometry and bumper-aware collision guarding. LiDAR detection covers previously observed free space along the indoor and outdoor route. Camera pixels and incident metadata are saved locally.

| Husky check | Completed laps | Anomalies | Collision stops | Result |
|---|---:|---:|---:|---|
| Outdoor + indoor objects, GUI | 2 | 2 | 0 | PASS; two camera PNGs and JSON records |
| Unchanged scene, GUI | 2 | 0 | 0 | PASS |
| Blocking object | 1 | 1 | 1 | PASS; second lap held before contact |
| Final sensor mounts, GUI preview | — | — | — | PASS; rendered model inspected |

An independent check found zero footprint overlaps in 937 measured Husky poses. Its front bumper stopped about **0.308 m** short of the blocking pallet. The route and doorway were retained.

Turn logs show **85.88–89.00°** of chassis rotation at quarter-turn transitions and **177.19–178.04°** at U-turn transitions. The controller releases forward movement only after heading error falls below 0.045 rad; the maximum logged residual was **2.53°**. Small arrival offsets and continued heading correction explain the difference from exact geometric 90°/180° turns. The old 45° observation was not reproduced; the old wheel-odometry setup could misestimate skid-steer rotation, but the historical cause is not proven.

Both mixed-test incidents were located within 0.5 m of their inserted objects, and each saved a synchronized 960×720 robot-camera frame. The detector used neither spawn coordinates nor camera classification to trigger those incidents. Camera images were visually inspected. The last visual pass added the sensor brackets; their collision geometry stays inside the guarded footprint and outside the LiDAR plane.

**14 offline tests pass**, including outdoor and indoor reference changes, multiple objects, moved-location association, unknown/occluded-space rejection, unchanged structure with pose noise, image byte decoding, camera projection and Husky drive/sensor calibration. The Husky and world SDF files validate.

- [Husky measured results](validation/husky_summary.json)
- [Husky GUI model](validation/husky_final_model.png)
- [Mixed-test events and camera paths](validation/husky_mixed_events.jsonl)
- [Clear-scene events and measured turns](validation/husky_clear_events.jsonl)
- [Blocking-test events and image path](validation/husky_blocking_events.jsonl)
- [Current measured route plot](validation/route_verification.png)

Camera projection is an estimate using nominal level-robot geometry, not visual confirmation. Off-axis incidents may have context images only. Detection uses a single LiDAR plane and needs a clean first lap; incident locations within 0.9 m of a previously reported location are deduplicated for the run. No remote operator control interface, semantic camera AI or privacy processing is implemented.

