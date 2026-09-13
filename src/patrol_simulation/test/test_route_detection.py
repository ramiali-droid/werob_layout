import math
from types import SimpleNamespace

from PIL import Image

from patrol_simulation.route_detection import RouteComparator, IncidentRegistry
from patrol_simulation.incident_evidence import camera_projection, save_camera_frame
from test_patrol_geometry import boxes, ray_scan
from test_patrol_geometry import PACKAGE
import xml.etree.ElementTree as ET


def confirmed(model, pose, obstacles):
    scan = ray_scan(pose, obstacles)
    assert model.compare_all(pose, scan) == []
    assert model.compare_all(pose, scan) == []
    return model.compare_all(pose, scan)


def test_outdoor_object_uses_the_same_reference_comparison_as_indoor():
    for pose, pallet in [((6,0,0), (8,-0.85,0,.4,.1)),
                         ((-2,-5,math.pi), (-3.5,-5.7,0,.4,.1))]:
        model = RouteComparator()
        model.learn(pose, ray_scan(pose, boxes()))
        result = confirmed(model, pose, boxes()+[pallet])
        assert len(result) == 1
        assert math.dist((result[0]['object_x_m'],result[0]['object_y_m']),pallet[:2]) < 0.5


def test_two_simultaneous_objects_and_another_location_in_same_lap():
    model = RouteComparator()
    pose = (6,0,0)
    model.learn(pose, ray_scan(pose, boxes()))
    objects = [(8,-.85,0,.4,.1), (8,.85,0,.4,.1)]
    results = confirmed(model, pose, boxes()+objects)
    assert len(results) == 2
    registry = IncidentRegistry()
    for result in results:
        assert registry.register(result['object_x_m'],result['object_y_m'])
        assert not registry.register(result['object_x_m']+.1,result['object_y_m'])
    # A relocated object beyond the association radius is a new location incident.
    assert registry.register(10,-.85)


def test_unchanged_outdoor_structure_and_pose_noise_do_not_alarm():
    model = RouteComparator()
    for pose in [(6,0,0), (12,-3,-math.pi/2), (-3,-5,math.pi)]:
        model.learn(pose, ray_scan(pose, boxes()))
        perturbed = (pose[0]+.04,pose[1]+.03,pose[2]+.015)
        for _ in range(4):
            assert not model.compare_all(perturbed,ray_scan(perturbed, boxes()))


def test_unobserved_or_occluded_space_is_not_assumed_clear():
    model = RouteComparator()
    pose = (6,0,0)
    wall = (7,0,0,.1,3)
    model.learn(pose, ray_scan(pose, [wall]))
    assert not model.novel(8,0)
    assert not model.novel(100,100)
    # Removing an occluder does not justify calling previously hidden space anomalous.
    for _ in range(4):
        assert not model.compare_all(pose,ray_scan(pose,[(8,0,0,.4,.1)]))


def test_camera_projection_distinguishes_object_view_from_context():
    assert camera_projection((0,0,0),(3,0),640,480) is not None
    assert camera_projection((0,0,0),(-3,0),640,480) is None
    assert camera_projection((0,0,0),(0,3),640,480) is None
    assert camera_projection((0,0,math.pi),(-3,0),640,480) is not None


def test_saved_camera_pixels_respect_encoding_and_row_padding(tmp_path):
    frame = SimpleNamespace(width=2,height=2,encoding='bgr8',step=8,
                            data=bytes([0,0,255, 0,255,0, 0,0, 255,0,0, 255,255,255, 0,0]))
    path = tmp_path/'camera.png'
    save_camera_frame(frame,path)
    with Image.open(path) as image:
        assert image.size == (2,2)
        assert list(image.getdata()) == [(255,0,0),(0,255,0),(0,0,255),(255,255,255)]


def test_husky_drive_and_sensor_calibration_match_the_patrol():
    from patrol_simulation.patrol_logic import LIDAR_X, HALF_LENGTH, HALF_WIDTH
    model = ET.parse(PACKAGE/'models/husky_patrol/model.sdf').find('model')
    drive = model.find("plugin[@name='gz::sim::systems::DiffDrive']")
    left = model.find("joint[@name='front_left_wheel']/pose").text.split()
    right = model.find("joint[@name='front_right_wheel']/pose").text.split()
    assert math.isclose(float(left[1])-float(right[1]),float(drive.findtext('wheel_separation')))
    for wheel in drive.findall('left_joint')+drive.findall('right_joint'):
        joint = model.find(f"joint[@name='{wheel.text}']")
        link = model.find(f"link[@name='{joint.findtext('child')}']")
        assert math.isclose(float(link.findtext('collision/geometry/cylinder/radius')),
                            float(drive.findtext('wheel_radius')))
    assert model.findtext("plugin[@name='gz::sim::systems::OdometryPublisher']/odom_topic") == '/odom'
    assert drive.findtext('odom_topic') != '/odom'
    lidar = list(map(float, model.findtext("link[@name='lidar_link']/pose").split()))
    camera = list(map(float, model.findtext("link[@name='camera_link']/pose").split()))
    assert math.isclose(lidar[0],LIDAR_X)
    assert math.isclose(camera[0],0.43)
    assert math.isclose(camera[2]-lidar[2],0.12)
    assert math.isclose(float(model.findtext("link[@name='camera_link']/sensor/camera/horizontal_fov")),1.2)
    assert HALF_LENGTH >= .48+.15/2
    assert HALF_WIDTH >= .2854+.1143/2
    for uri in model.findall('.//mesh/uri'):
        assert (PACKAGE/'models'/uri.text.removeprefix('model://')).is_file()
