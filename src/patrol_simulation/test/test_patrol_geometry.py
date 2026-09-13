"""Independent SDF ray casting checks the route and live detector geometry."""
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from patrol_simulation.patrol_logic import (
    WAYPOINTS, HALF_LENGTH, HALF_WIDTH, ReferenceComparator, collision_risk, scan_usable,
)

PACKAGE = Path(__file__).resolve().parents[1]


def boxes():
    result = []
    for model in ET.parse(PACKAGE / 'worlds/pilot_world.sdf').findall('.//world/model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                size = collision.findtext('geometry/box/size')
                if size:
                    x, y, z, _, _, yaw = map(float, link.findtext('pose', '0 0 0 0 0 0').split())
                    sx, sy, sz = map(float, size.split())
                    # Ignore the doorway lintel and caps above the entire rover.
                    if z+sz/2 > 0.05 and z-sz/2 < 0.70:
                        result.append((x, y, yaw, sx/2, sy/2))
    return result


def ray_scan(pose, obstacles):
    x, y, yaw = pose
    x, y = x+0.12*math.cos(yaw), y+0.12*math.sin(yaw)
    ranges = []
    for i in range(720):
        angle = -math.pi + i * 2*math.pi/719 + yaw
        nearest = math.inf
        for bx, by, byaw, hx, hy in obstacles:
            c, s = math.cos(byaw), math.sin(byaw)
            ox, oy = c*(x-bx)+s*(y-by), -s*(x-bx)+c*(y-by)
            dx, dy = math.cos(angle-byaw), math.sin(angle-byaw)
            low, high = 0.0, 25.0
            for origin, direction, half in ((ox, dx, hx), (oy, dy, hy)):
                if abs(direction) < 1e-10:
                    if abs(origin) > half:
                        high = -1
                        break
                else:
                    ends = sorted(((-half-origin)/direction, (half-origin)/direction))
                    low, high = max(low, ends[0]), min(high, ends[1])
            if low <= high and low > 0.12:
                nearest = min(nearest, low)
        ranges.append(nearest)
    return SimpleNamespace(ranges=ranges, angle_min=-math.pi, angle_increment=2*math.pi/719,
                           range_min=0.12, range_max=25.0)


def overlap(a, b):
    # Separating axis theorem: rectangles intersect iff no separating axis exists.
    ax, ay, aa, ahx, ahy = a
    bx, by, ba, bhx, bhy = b
    for direction in (aa, aa+math.pi/2, ba, ba+math.pi/2):
        separation = abs((bx-ax)*math.cos(direction)+(by-ay)*math.sin(direction))
        radius = (ahx*abs(math.cos(aa-direction)) + ahy*abs(math.sin(aa-direction))
                  + bhx*abs(math.cos(ba-direction)) + bhy*abs(math.sin(ba-direction)))
        if separation > radius:
            return False
    return True


def test_accepted_route_and_turns_clear_actual_sdf_collisions():
    obstacles = boxes() + [(-3.5, -5.7, 0.0, 0.4, 0.1)]
    for start, end in zip(WAYPOINTS, WAYPOINTS[1:]):
        dx, dy = end[1]-start[1], end[2]-start[2]
        yaw = math.atan2(dy, dx)
        for i in range(math.ceil(math.hypot(dx, dy)/0.05)+1):
            ratio = min(1.0, i*0.05/math.hypot(dx, dy))
            footprint = (start[1]+dx*ratio, start[2]+dy*ratio, yaw, HALF_LENGTH+0.05, HALF_WIDTH+0.05)
            assert not any(overlap(footprint, b) for b in obstacles), (start, end, footprint)
    # Actual corners only; the collinear bottleneck waypoints do not require rotation.
    for x, y in ((0,0), (12,0), (12,-5), (-5.5,-5), (5,-5), (5,0)):
        for i in range(72):
            assert not any(overlap((x,y,i*math.pi/36,HALF_LENGTH+0.05,HALF_WIDTH+0.05), b) for b in obstacles)


def test_guard_allows_clear_bottleneck_but_stops_for_blocking_object():
    assert collision_risk(ray_scan((1.5,-5,math.pi), boxes()), 0.75) is None
    assert collision_risk(ray_scan((-2.4,-5,math.pi), boxes()+[(-3.5,-5,0,0.4,0.1)]), 0.75) is not None
    assert collision_risk(ray_scan((-3.5,-5,math.pi), boxes()+[(-3.5,-5.7,0,0.4,0.1)]), 0.75) is None


def test_pose_matched_reference_has_no_clear_lap_alarm():
    model = ReferenceComparator((-4.3,-2.7,-6.1,-4))
    for x in (-2.0,-2.5,-3.0,-3.5,-4.0,-4.5):
        model.learn((x,-5,math.pi), ray_scan((x,-5,math.pi),boxes()))
    for x in (-2.0,-2.5,-3.0,-3.5,-4.0,-4.5):
        for _ in range(3):
            assert model.compare((x+0.05,-4.97,math.pi-0.02),ray_scan((x+0.05,-4.97,math.pi-0.02),boxes())) is None


def test_new_object_requires_multiple_scans_and_reports_its_position():
    model = ReferenceComparator((-4.3,-2.7,-6.1,-4))
    pose = (-3.3,-5,math.pi)
    model.learn(pose, ray_scan(pose, boxes()))
    scan = ray_scan(pose, boxes()+[(-3.5,-5.7,0,0.4,0.1)])
    assert model.compare(pose,scan) is None
    assert model.compare(pose,scan) is None
    result = model.compare(pose,scan)
    assert result and result['difference_m'] > 0.5
    assert -3.9 <= result['object_x_m'] <= -3.1
    assert abs(result['object_y_m']+5.6) < 0.05
    # Returning through the same location with opposite heading is a different reference.
    assert model.compare((-3.3,-5,0),scan) is None


def test_lidar_plane_intersects_pallet_above_its_own_chassis():
    model = ET.parse(PACKAGE / 'models/husky_patrol/model.sdf').find('model')
    model_z = float(model.findtext('pose').split()[2])
    lidar_z = model_z+float(model.find("link[@name='lidar_link']/pose").text.split()[2])
    chassis = model.find("link[@name='base_link']")
    top = max(model_z+float(c.findtext('pose', '0 0 0 0 0 0').split()[2])
              +float(c.findtext('geometry/box/size').split()[2])/2 for c in chassis.findall('collision')
              if c.attrib['name'].startswith('base_link_collision'))
    pallet_height = float(ET.parse(PACKAGE/'models/unexpected_pallet/model.sdf').findtext('.//collision/geometry/box/size').split()[2])
    assert top < lidar_z < pallet_height
    assert model.findtext('static') == 'false'


def test_invalid_scan_is_not_treated_as_clear_space():
    scan = ray_scan((0,0,0), [])
    assert scan_usable(scan)  # +inf is a valid no-return measurement.
    scan.ranges = [math.nan] * 720
    assert not scan_usable(scan)
    # Reproduced on the desktop Intel/Ogre renderer: every ray at the near plane.
    scan.ranges = [scan.range_min] * 720
    assert not scan_usable(scan)
    # A few near returns do not invalidate the rest of a legitimate scene scan.
    scan.ranges[-1] = 3.0
    assert scan_usable(scan)


def test_doorway_joins_both_walls_and_preserves_clear_opening():
    model = ET.parse(PACKAGE / 'worlds/pilot_world.sdf').find(".//model[@name='indoor_passage']")

    def extent(name, axis):
        link = model.find(f"link[@name='{name}']")
        centre = float(link.findtext('pose').split()[axis])
        size = float(link.findtext('collision/geometry/box/size').split()[axis])
        return centre-size/2, centre+size/2

    # No side gaps or wall overlap; jambs join the panels face-to-face.
    assert math.isclose(extent('bottleneck_left', 1)[1], extent('north_wall', 1)[0])
    assert math.isclose(extent('bottleneck_right', 1)[0], extent('south_wall', 1)[1])
    assert math.isclose(extent('bottleneck_left', 1)[0], extent('door_jamb_north', 1)[1])
    assert math.isclose(extent('bottleneck_right', 1)[1], extent('door_jamb_south', 1)[0])
    north = extent('door_jamb_north', 1)[0]
    south = extent('door_jamb_south', 1)[1]
    assert math.isclose(north-south, 1.2)
    assert math.isclose((north+south)/2, -5.0)
    assert math.isclose(extent('door_header', 2)[0], 2.05)
