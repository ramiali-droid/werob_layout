# Autonomous Patrol Pilot — integration-engineering response

## Assumptions

1. The site has a lawful security-surveillance basis and has completed the required local privacy assessment; this is not legal advice.
2. “Eight laps in 24 hours” means eight scheduled attempts every calendar day, including a 30–31-day month. The contractual interpretation of a partial lap must be agreed: below I count a lap only when its planned route is completed.
3. The robot has a safe-stop capability, authenticated software updates, onboard time synchronisation, and a remotely accessible telemetry API. These are acceptance prerequisites, not facts supplied by the case.

## 1. Mapping and localisation concept

I would first walk and scan the site in operating conditions, collecting multiple LiDAR/camera passes by day and night. From these I build a **versioned 3-D base map**, with separate semantic layers: (a) permanent structure—building shell, columns, curb edges, gate posts and dock; (b) navigation layer—drivable corridors, slope/curb/cable-bridge limits and route centre-lines; (c) dynamic layer—storage bays, vehicle areas and other places explicitly expected to change. A supervisor accepts the baseline map, and every map or route edit gets an approver, time, reason, and rollback version.

Indoors, localisation is LiDAR scan matching against the structural layer, fused with wheel/IMU odometry; fixed, surveyable reflectors or AprilTags are recovery anchors at long plain corridors, junctions and the dock. Outdoors, I fuse LiDAR/visual odometry/IMU with RTK GNSS where sky visibility permits; the perimeter posts and dock remain LiDAR anchors when GNSS degrades. The fusion publishes pose covariance: at a defined threshold the robot slows, re-localises at the nearest anchor, and then takes a pre-approved escape route or pauses—not “guessing” its location.

I define exclusion zones before mapping routes: stairs, cable bridges above the platform rating, curb/drop-off margins, forklift manoeuvre areas, public paths, blind corners, charging-station clearance and all fire/emergency egress. Geofences are inflated by robot footprint plus uncertainty and stopping distance. Each long corridor and outdoor segment has an escape node: reverse to the last confirmed anchor, pull into a marked refuge bay, or return to dock; no escape path crosses a pedestrian-only or emergency route.

The reference map is not a single “everything must stay the same” image. Detection is limited to **approved monitored volumes**: e.g. gate closed/open state, clear egress, no object in a bottleneck, protected-door state, or a vehicle-free security lane. Daily-change areas are masked or modelled as “expected occupancy,” and new items must persist across N observations / a dwell time before an event. The operator can create a time-bounded authorised change (“pallet in bay B until 06:00”); it is auditable, expires automatically, and never changes a perimeter or safety rule.

Two honest weak points:

1. **Indoor feature-poor or repetitious corridors** can cause LiDAR aliasing, especially when shelving is moved. I mitigate with surveyed recovery anchors, map confidence gates, speed reduction and a deterministic retreat-to-anchor policy; I do not allow autonomous passage below a pose-confidence threshold.
2. **Outdoor wet snow, standing water and low sun / snow cover** can degrade LiDAR returns, stereo perception and GNSS multipath. I mitigate with weather-derived operating limits, lens/radome checks, thermal as corroboration rather than sole navigation, redundant structural LiDAR landmarks, and a “no-go / return dock” state. Pilot acceptance must prove those limits in the actual winter conditions.

### Sketch (logical layers)

```text
                 OUTDOOR YARD / RTK + LiDAR landmarks
 [dock] --- (yard waypoint) --- [GATE: monitored closed/open]
   |                                      |
   +---- indoor entry ---- [1.1m BOTTLENECK: clear/blocked] --- indoor checkpoint
          LiDAR anchors       refuge / reverse-to-anchor          LiDAR anchors

 permanent structure → localisation reference
 navigation/route layer → approved centre-lines, geofences, escape nodes
 dynamic layer → storage zones masked unless a monitored volume is violated
```

## 2. Prototype

The working prototype is in this repository. [README](README.md) gives the start commands and a three-minute recording plan. The scene is deliberately self-contained so it does not depend on downloaded Fuel assets. It includes the yard, dock, open red gate, indoor passage, amber bottleneck, a repeating route, and machine-readable JSON events. The route returns through the same east-side indoor opening rather than crossing a wall. Its transparent waypoint controller proves the requested map → route → detection chain; it is **not** presented as production navigation or perception.


## 3. THE EVENT INTERFACE TO THE CONTROL CENTER: Design & Architecture

Transport Strategy (The "No Separate Window" Requirement)
The JSON payloads (provided in event_examples.json) serve as the Canonical Event. To ensure the dispatcher never has to open a separate window, this payload is routed via a three-tiered transport strategy:

    Tier 1 (Genetec Connector): The preferred method. The payload fields map directly to Genetec's native UI, allowing the dispatcher to triage within their existing dashboard.

    Tier 2 (HTTPS Webhook): If the connector is unavailable, the same JSON is sent via an authenticated webhook to the client's IT backend to trigger a native pop-up.

    Tier 3 (SIA DC-09): If IP webhooks are unsupported by the client's legacy dispatch software, the robot maps the event to an ANSI-standard Contact ID code (e.g., E130 for Intrusion) to ring the physical alarm bell, while the JSON is safely logged in the background.

Design Justification (The 5 Sentences)
To enable a dispatcher to confidently deploy a guard within ten seconds, the payload prioritizes immediate context—severity, exact coordinates, observed-versus-reference state, and a recommended action—mapped directly into native Genetec alert banners. To enforce the strict 72-hour retention and privacy requirements, evidence is delivered purely as short-lived, protected URIs, with faces dynamically redacted at the edge before transmission so raw PII never enters the dispatch system. Unredacted originals remain safely siloed on the robot's local storage, retrievable only via a documented, case-linked audit workflow. To serve as an unalterable proof of service for the client's billing department, the completed-round canonical JSON is "JOHN" hashed and signed by a device-managed private key. Consistent with NIST guidelines, each report includes the previous report’s hash to create a tamper-evident cryptographic chain, ensuring the client can mathematically prove the patrol logs were neither manually altered nor retroactively deleted.

(Note: See event_examples.json for the exact payload structures for both an anomaly alarm and a completed patrol round.)


---

### 4. AVAILABILITY AND FALSE ALARMS: The Numerical Part

**Part 1: Availability Calculations & Error Budgets**

| Assumed calendar month | Planned laps (8/day) | 95% minimum completed | Loss budget before credit |
| --- | --- | --- | --- |
| 30 days | 240 | 228 | 12 |
| 31 days | 248 | 236 (ceiling of 235.6) | 12 |

**Operational Math & Battery Safety Constraints:**
To achieve 8 laps in 24 hours without violating safety battery reserves (crucial in winter when cold temperatures degrade Li-ion capacity), the system must operate on the following strict cycle:

* **Average Speed:** ~2.0 km/h over mixed terrain (gravel, stairs, corridors).
* **Time per Lap:** 3 km / 2.0 km/h = **1.5 hours per lap**.
* **Per Cycle (2 Laps):** 2 laps × 1.5h = **3 hours of active driving**. This leaves a **1-hour (25%) safety reserve** out of the 4-hour battery capacity to handle unexpected detours, heavy gravel drag, or cold-weather voltage sag.
* **Charging Time:** 90-minute (1.5h) autonomous dock session.
* **Total Cycle Time:** 3 hours driving + 1.5 hours charging = **4.5 hours per cycle**.
* **Daily Total:** 4 cycles per day × 2 laps = **8 laps**, consuming **18 hours total** (12h driving + 6h charging).

This leaves a healthy **6-hour daily buffer** in the 24-hour schedule, which we desperately need to absorb minor delays, dock retries, or weather holds without breaking our 12-lap monthly error budget.

**SLA Protection Strategies:**

1. **Excused Omissions (Client Faults):** If a lap is aborted because a client forklift fully blocks a corridor or unplowed snow traps the dock, it is flagged as an Excused Omission and does not deduct from our 12-lap fault budget.
2. **Dynamic Truncation:** If a path is blocked midway, the robot aborts only that segment rather than stalling out, preserving battery and schedule integrity.

**Top 3 Technical Budget Consumers & Mitigations:**

1. **Charge/dock failures and battery margin errors:** Cold-reduced capacity or dirty contacts. *Mitigation:* Pre-departure energy estimates with weather reserves; strict 20% state-of-charge return-to-dock triggers; and two rapid dock-alignment retries before alerting human staff.
2. **Weather/sensor degradation:** Snow/rain occludes optics or produces poor LiDAR scans, causing safe stops. *Mitigation:* Heated sensor windows, sensor-health telemetry, statistical point-cloud filtering (SOR) for snow, and a conservative weather operating envelope (safety weather holds count as Excused Omissions).
3. **Cellular/control link loss:** Multi-provider SIM does not remove local structural dead-zones. *Mitigation:* Buffered store-and-forward telemetry with idempotency. The robot relies on local safe-autonomy to navigate through dead-zones and bursts the cached alarm payloads the moment the cellular link is re-established.

### Five-night false-alarm reduction

I start from labelled raw candidate observations, not from “turn down sensitivity.” Night 1 records baseline candidates with video/thermal/LiDAR confidence, zone, weather, lighting and ground truth; the week-one KPI is **false dispatchable alarms per completed patrol lap**, paired with detection/recall for seeded safety/perimeter events. Nights 2–3 adjust only zone geometry, persistence/dwell time and multi-sensor corroboration after reviewing false positives by cause. Night 4 repeats a fixed challenge set: open gate, a new pallet in the monitored bottleneck, person/vehicle at a protected perimeter, and authorised normal changes; measure missed/late alarms as well as false alarms. Night 5 freezes the candidate configuration only if recall and maximum detection latency meet the agreed acceptance criteria; otherwise retain the safer setting, document the residual false alarms, and extend calibration rather than hide them.

To lower false alarms during the pre-go-live window without compromising security or "blinding" the robot, we reject arbitrary sensor sensitivity adjustments. Instead, we use a systematic, data-driven 5-night calibration protocol grounded in field robotics deployment frameworks.

**The 5-Night Calibration Procedure**

* **Night 1 (Baseline Logging):** Run full patrol schedules with all automated alarms routed to an offline engineering log rather than the dispatch center. Capture raw candidate observations containing video, thermal, and LiDAR confidence scores alongside environmental metadata (weather, lighting, and spatial zones) to identify root causes.
* **Nights 2–3 (Spatial-Temporal Tuning):** Eliminate false positives by adjusting logic layers rather than hardware settings. Apply precise geometric polygon masks to ignore chronic environmental triggers (e.g., upper-frame trees or steam vents) and enforce *temporal dwell time*—requiring an anomaly to persist across consecutive frames and waypoints rather than triggering on single-frame transient noise.
* **Night 4 (Seeded Challenge Set):** Subject the robot to a repeatable physical stress test. Introduce controlled anomalies: an open gate, a newly restacked pallet in a monitored bottleneck, and a thermal target at the perimeter. Measure both false positives and false negatives (missed detections) under identical conditions.
* **Night 5 (Safe Fallback Freeze):** Freeze the candidate configuration *only* if detection recall and maximum latency meet acceptance criteria. If a balance cannot be safely struck, default to the safer setting, document the residual noise, and extend calibration rather than risking a blind spot.

**Week One KPI Focus**
The primary metric monitored during week one is **false dispatchable alarms per completed patrol lap**, paired strictly with a **Detection Recall Rate** for seeded safety and perimeter events.

* *Why:* Tracking false alarms alone creates a dangerous incentive to turn down sensor thresholds until the robot reports zero events (blinding it). Pairing this metric with a minimum recall threshold ensures that while we protect the dispatcher from alert fatigue, the robot maintains its structural integrity as a reliable security asset.
