#!/usr/bin/env python3

import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'moving_turtle'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
	(share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
	(share_dir + '/param', glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeon',
    maintainer_email='prizehun@yonsei.ac.kr',
    description='TODO: Package description',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'RVD_publisher = moving_turtle.RVD_publisher:main',
        'RVD_subscriber = moving_turtle.RVD_subscriber:main'
        ],
    },
)
