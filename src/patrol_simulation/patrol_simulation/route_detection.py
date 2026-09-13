"""Route-wide LiDAR change detection in space observed during the reference lap."""
import math

from .patrol_logic import LIDAR_X, ReferenceComparator, scan_points, wrap


class RouteComparator(ReferenceComparator):
    def __init__(self, threshold=0.5, persistence=3, resolution=0.15):
        super().__init__(None, threshold, persistence)
        self.resolution = resolution
        self.free = set()
        self.occupied = set()
        self.tracks = []

    def cell(self, x, y):
        return math.floor(x/self.resolution), math.floor(y/self.resolution)

    def learn(self, pose, scan):
        count = len(self.samples)
        super().learn(pose, scan)
        if len(self.samples) == count:
            return
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        for _, px, py in scan_points(scan):
            self.occupied.add(self.cell(x+c*px-s*py, y+s*px+c*py))
        ox, oy = x+LIDAR_X*c, y+LIDAR_X*s
        # Subsample rays, not poses, when recording known free space.
        for i in range(0, len(scan.ranges), 3):
            distance = scan.ranges[i]
            if math.isinf(distance) and distance > 0:
                distance = scan.range_max
            if not math.isfinite(distance) or distance < 0.6:
                continue
            angle = scan.angle_min+i*scan.angle_increment+yaw
            dx, dy = math.cos(angle), math.sin(angle)
            for step in range(3, int((min(distance, scan.range_max)-0.3)/self.resolution)):
                r = step*self.resolution
                self.free.add(self.cell(ox+r*dx, oy+r*dy))

    def novel(self, x, y):
        cx, cy = self.cell(x, y)
        if (cx, cy) not in self.free:
            return False
        # Ignore the uncertainty band around baseline walls and other surfaces.
        return not any((cx+dx, cy+dy) in self.occupied
                       for dx in range(-2, 3) for dy in range(-2, 3))

    def _miss(self):
        self.tracks = []
        return super()._miss()

    def compare_all(self, pose, scan):
        x, y, yaw = pose
        matches = [(math.hypot(x-r.pose[0], y-r.pose[1]), r) for r in self.samples
                   if abs(wrap(yaw-r.pose[2])) < 0.12]
        if not matches:
            self._miss()
            return []
        distance, reference = min(matches, key=lambda pair: pair[0])
        if distance > 0.24:
            self._miss()
            return []
        groups, group, last_index = [], [], -2
        c, s = math.cos(yaw), math.sin(yaw)
        for i, px, py in scan_points(scan):
            wx, wy = x+c*px-s*py, y+s*px+c*py
            bearing = wrap(scan.angle_min+i*scan.angle_increment+yaw-reference.pose[2])
            j = round((bearing-reference.angle_min)/reference.angle_increment)
            baseline = reference.ranges[j] if 0 <= j < len(reference.ranges) else math.nan
            if math.isinf(baseline) and baseline > 0:
                baseline = reference.range_max
            changed = (math.isfinite(baseline) and baseline-scan.ranges[i] > self.threshold
                       and self.novel(wx, wy))
            if not changed or i != last_index+1 or (group and math.dist((wx, wy), group[-1][:2]) > 0.35):
                if len(group) >= 4:
                    groups.append(group)
                group = []
            if changed:
                group.append((wx, wy, baseline, scan.ranges[i]))
            last_index = i
        if len(group) >= 4:
            groups.append(group)

        previous, self.tracks, results = list(self.tracks), [], []
        for points in groups:
            centre = tuple(sum(p[k] for p in points)/len(points) for k in range(4))
            match = min(previous, key=lambda t: math.dist(centre[:2], t[0]), default=None)
            count = 1
            if match and math.dist(centre[:2], match[0]) < 0.6:
                count = match[1]+1
                previous.remove(match)
            self.tracks.append((centre[:2], count))
            if count >= self.persistence:
                results.append({'object_x_m': round(centre[0], 2), 'object_y_m': round(centre[1], 2),
                                'reference_range_m': round(centre[2], 2), 'observed_range_m': round(centre[3], 2),
                                'difference_m': round(centre[2]-centre[3], 2),
                                'consecutive_scans': count, 'changed_beams': len(points),
                                'reference_pose_distance_m': round(distance, 3)})
        return results


class IncidentRegistry:
    """Suppress repeat views of a location, allowing several incidents in one lap."""
    def __init__(self, distance=0.9):
        self.distance = distance
        self.locations = []

    def register(self, x, y):
        if any(math.dist((x, y), previous) < self.distance for previous in self.locations):
            return False
        self.locations.append((x, y))
        return True
