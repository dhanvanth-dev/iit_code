import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aquavision'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    # IMPORTANT: ROS 2 Humble OpenCV requires numpy<2.0
    install_requires=[
        'setuptools',
        'numpy<2.0',
    ],
    zip_safe=True,
    maintainer='dhanvanth',
    maintainer_email='dhanvanthkrishnan@gmail.com',
    description='Aquavision 2.0 – ROS 2 Autonomous Gate Traversal System',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'vision_node = aquavision.vision_node:main',
            'nav_bridge_node = aquavision.nav_bridge_node:main',
            'mission_manager = aquavision.mission_manager:main',
        ],
    },
)
