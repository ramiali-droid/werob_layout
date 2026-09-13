"""Patrol with physical collisions, sensor comparison and lap-triggered test object."""
import os
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('patrol_simulation')
    gui = LaunchConfiguration('gui')
    sim_params = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open Gazebo GUI.'),
        DeclareLaunchArgument('headless_rendering', default_value='false', description='Use EGL instead of an X display.'),
        DeclareLaunchArgument('render_engine', default_value='ogre', choices=['ogre2', 'ogre']),
        DeclareLaunchArgument('software_rendering', default_value='true',
                              description='Use Mesa llvmpipe; the local Intel/Ogre driver returns invalid LiDAR ranges.'),
        DeclareLaunchArgument('obstacle_mode', default_value='beside', choices=['beside', 'blocking', 'outdoor', 'mixed', 'none']),
        DeclareLaunchArgument('evidence_dir', default_value=os.path.join(os.getcwd(), 'incidents')),
        DeclareLaunchArgument('max_speed', default_value='0.75'),
        DeclareLaunchArgument('controller', default_value='true', description='Disable only for isolated physics tests.'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1', condition=IfCondition(LaunchConfiguration('software_rendering'))),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe', condition=IfCondition(LaunchConfiguration('software_rendering'))),
        # Keep Qt from loading incompatible desktop style plugins inherited from Snap editors.
        *[UnsetEnvironmentVariable(key) for key in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GIO_MODULE_DIR')
          if '/snap/' in os.environ.get(key, '')],
        SetEnvironmentVariable('QT_QPA_PLATFORM', os.environ.get('QT_QPA_PLATFORM', 'xcb')),
        SetEnvironmentVariable('QT_STYLE_OVERRIDE', os.environ.get('QT_STYLE_OVERRIDE', 'Fusion')),
        SetEnvironmentVariable('GZ_PARTITION', os.environ.get('GZ_PARTITION') or 'werob_' + uuid4().hex),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models') + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': [PythonExpression(["'-r ' if '", gui, "' == 'true' else '-r -s '"]),
                                          PythonExpression(["'--headless-rendering ' if '", LaunchConfiguration('headless_rendering'), "' == 'true' else ''"]),
                                          '--render-engine ', LaunchConfiguration('render_engine'), ' ',
                                          '--gui-config ', os.path.join(share, 'config/patrol_gui.config'), ' ',
                                          os.path.join(share, 'worlds/pilot_world.sdf')],
                              'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'patrol_pilot', '-file', os.path.join(share, 'models/husky_patrol/model.sdf'),
                        '-name', 'husky_patrol', '-x', '0', '-y', '0', '-z', '0.14'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': os.path.join(share, 'config/bridge.yaml')}], output='screen'),
        Node(package='patrol_simulation', executable='patrol_controller',
             parameters=[sim_params, {'max_speed': ParameterValue(LaunchConfiguration('max_speed'), value_type=float)}],
             condition=IfCondition(LaunchConfiguration('controller')), output='screen'),
        Node(package='patrol_simulation', executable='anomaly_detector',
             parameters=[sim_params, {'evidence_dir': LaunchConfiguration('evidence_dir')}], output='screen'),
        Node(package='patrol_simulation', executable='scenario_manager',
             parameters=[sim_params, {'obstacle_mode': LaunchConfiguration('obstacle_mode')}], output='screen'),
    ])
