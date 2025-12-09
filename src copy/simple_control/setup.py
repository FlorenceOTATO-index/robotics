from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simple_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_maker = simple_control.map_maker:main',
            'local_planner = simple_control.local_planner:main',
            'global_planner = simple_control.global_planner:main',
            'final_path = simple_control.final_path:main',
            'tower_to_map = simple_control.tower_to_map:main',
            'rescue_mission = simple_control.rescue_mission:main'
        ],
    },
)
