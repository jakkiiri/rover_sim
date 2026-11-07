from glob import glob

from setuptools import setup

package_name = 'rover_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    maintainer='silvio-vaso',
    maintainer_email='silviovaso1@gmail.com',
    description='A* path planning on a stereo-camera obstacle grid for rover_sim.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'astar_planner = rover_planner.planner_node:main',
            'ground_truth_odom = rover_planner.ground_truth_odom:main',
            'path_follower = rover_planner.path_follower:main',
            'validate_planner = rover_planner.validate:main',
            'slope_test = rover_planner.slope_test:main',
            'validate_terrain = rover_planner.validate_terrain:main',
        ],
    },
)
