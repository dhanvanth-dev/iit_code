import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vauv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhanvanth',
    maintainer_email='dhanvanthkrishnan@gmail.com',
    description='VAUV Control Package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_node = vauv_control.mission_node:main',
            'logger_node = vauv_control.logger_node:main',
        ],
    },
)
