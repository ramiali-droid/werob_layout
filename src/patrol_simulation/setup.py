from setuptools import setup
from glob import glob
import os


package_name = 'patrol_simulation'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    ('share/' + package_name + '/config', glob('config/*.config')),
    ('share/' + package_name + '/reference', glob('reference/*.yaml')),
]
# Function to add recursive directories (e.g., models/)
def add_recursive_files(base_dir):
    for root, dirs, files in os.walk(base_dir):
        dirs[:] = [d for d in dirs if not d.startswith('.') and d != '__pycache__']
        file_list = [os.path.join(root, f) for f in files]
        if file_list:  # Only add if there are files
            target_dir = os.path.join('share', package_name, root)
            data_files.append((target_dir, file_list))

add_recursive_files('models')

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'patrol_controller = patrol_simulation.patrol_controller:main',
            'scenario_manager = patrol_simulation.scenario_manager:main',
            'anomaly_detector = patrol_simulation.anomaly_detector:main',
        ],
    },
)
