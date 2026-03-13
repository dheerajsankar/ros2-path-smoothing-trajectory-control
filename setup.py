import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dsn',
    maintainer_email='dheerajsankar2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "waypoints = navigation.waypoints:main",
            "path_smoothening = navigation.path_smoothening:main",
            "trajectory_gen = navigation.trajectory_gen:main",
            "controller = navigation.controller:main",
            "sim_robot = navigation.sim_robot:main",
            "traj_plotter = navigation.traj_plotter:main",
        ],
    },
)
