"""Geometry and scan comparison used by the live patrol and offline checks."""
import math
from dataclasses import dataclass


# The route accepted in the original thread. All coordinates are Gazebo world metres.
WAYPOINTS = [
    ('dock', 0.0, 0.0), ('yard_north', 7.5, 0.0), ('gate', 12.0, 0.0),
    ('yard_south', 12.0, -5.0), ('corridor_entry', 5.0, -5.0),
    ('bottleneck_in', 1.0, -5.0), ('indoor_checkpoint', -5.5, -5.0),
    ('bottleneck_out', 1.0, -5.0), ('corridor_exit', 5.0, -5.0),
    ('outdoor_return', 5.0, 0.0), ('dock', 0.0, 0.0),
]
LIDAR_X = 0.12
# Conservative Husky footprint including both bumpers and all four tyres.
HALF_LENGTH, HALF_WIDTH = 0.56, 0.35


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_xyyaw(pose):
    q = pose.orientation
    return (pose.position.x, pose.position.y,
            math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z)))


def scan_points(scan):
    for i, distance in enumerate(scan.ranges):
        if math.isfinite(distance) and scan.range_min <= distance < scan.range_max:
            angle = scan.angle_min + i * scan.angle_increment
            x, y = LIDAR_X + distance * math.cos(angle), distance * math.sin(angle)
            # Exclude returns inside the robot; actual body contact has its own sensor.
            if -HALF_LENGTH < x < HALF_LENGTH and abs(y) < HALF_WIDTH:
                continue
            yield i, x, y


def scan_usable(scan):
    return (len(scan.ranges) >= 30 and math.isfinite(scan.angle_increment)
            and scan.angle_increment > 0 and scan.range_max > scan.range_min >= 0
            and not scan_saturated(scan)
            and any((math.isfinite(r) and scan.range_min <= r < scan.range_max)
                    or (math.isinf(r) and r > 0) for r in scan.ranges))


def scan_saturated(scan):
    """A full ring stuck at the near clip plane is not a usable scene observation."""
    return bool(scan.ranges) and all(
        math.isfinite(r) and abs(r-scan.range_min) <= 1e-5 for r in scan.ranges)


def collision_risk(scan, speed, turning=False):
    """Check the swept footprint, not a cone that mistakes side walls for obstacles."""
    front_limit = HALF_LENGTH + 0.16 + max(0.0, speed) * 0.45
    for _, x, y in scan_points(scan):
        if turning:
            if math.hypot(x, y) < math.hypot(HALF_LENGTH, HALF_WIDTH) + 0.06:
                return math.hypot(x, y)
        elif -HALF_LENGTH <= x < front_limit and abs(y) < HALF_WIDTH + 0.08:
            return max(0.0, x - HALF_LENGTH)
    return None


@dataclass
class ReferenceScan:
    pose: tuple
    ranges: tuple
    angle_min: float
    angle_increment: float
    range_max: float


class ReferenceComparator:
    """Pose-matched first-lap scans; only changes inside the approved volume alarm."""
    def __init__(self, bounds, threshold=0.5, persistence=3):
        self.bounds = bounds
        self.threshold = threshold
        self.persistence = persistence
        self.samples = []
        self.consecutive = 0
        self.last_candidate = None

    def learn(self, pose, scan):
        if self.samples:
            last = self.samples[-1].pose
            if math.hypot(pose[0]-last[0], pose[1]-last[1]) < 0.12 and abs(wrap(pose[2]-last[2])) < 0.08:
                return
        self.samples.append(ReferenceScan(pose, tuple(scan.ranges), scan.angle_min,
                                          scan.angle_increment, scan.range_max))

    def compare(self, pose, scan):
        x, y, yaw = pose
        matches = [(math.hypot(x-r.pose[0], y-r.pose[1]), r) for r in self.samples
                   if abs(wrap(yaw-r.pose[2])) < 0.12]
        if not matches:
            return self._miss()
        distance, reference = min(matches, key=lambda pair: pair[0])
        if distance > 0.24:
            return self._miss()
        xmin, xmax, ymin, ymax = self.bounds
        candidates, group = [], []
        last_index = -2
        for i, local_x, local_y in scan_points(scan):
            world_x = x + local_x * math.cos(yaw) - local_y * math.sin(yaw)
            world_y = y + local_x * math.sin(yaw) + local_y * math.cos(yaw)
            bearing = wrap(scan.angle_min + i*scan.angle_increment + yaw-reference.pose[2])
            j = round((bearing-reference.angle_min) / reference.angle_increment)
            if not (0 <= j < len(reference.ranges)):
                continue
            baseline = reference.ranges[j]
            if math.isinf(baseline) and baseline > 0:
                baseline = reference.range_max
            observed = scan.ranges[i]
            if (xmin <= world_x <= xmax and ymin <= world_y <= ymax
                    and math.isfinite(baseline) and baseline-observed > self.threshold):
                if i != last_index + 1:
                    group = []
                group.append((world_x, world_y, baseline, observed))
                last_index = i
                if len(group) >= 4:
                    candidates = list(group)
            else:
                group = []
        if not candidates:
            return self._miss()
        centre = tuple(sum(c[k] for c in candidates)/len(candidates) for k in range(4))
        if self.last_candidate and math.hypot(centre[0]-self.last_candidate[0], centre[1]-self.last_candidate[1]) > 0.5:
            self.consecutive = 0
        self.last_candidate = centre
        self.consecutive += 1
        if self.consecutive < self.persistence:
            return None
        return {'object_x_m': round(centre[0], 2), 'object_y_m': round(centre[1], 2),
                'reference_range_m': round(centre[2], 2), 'observed_range_m': round(centre[3], 2),
                'difference_m': round(centre[2]-centre[3], 2),
                'consecutive_scans': self.consecutive, 'changed_beams': len(candidates),
                'reference_pose_distance_m': round(distance, 3)}

    def _miss(self):
        self.consecutive = 0
        self.last_candidate = None
        return None
