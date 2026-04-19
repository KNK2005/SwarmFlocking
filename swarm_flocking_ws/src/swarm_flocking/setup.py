from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'swarm_flocking'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Swarm Dev',
    maintainer_email='dev@example.com',
    description='Reynolds flocking with obstacle navigation for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boid_node = swarm_flocking.boid_node:main',
            'flock_monitor_node = swarm_flocking.flock_monitor_node:main',
            'physics_node = swarm_flocking.physics_node:main',
        ],
    },
)
