#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Ayumu Saito
# SPDX-License-Identifier: BSD-3-Clause

from setuptools import setup

package_name = 'twist_stdin_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ayumu Saito',
    maintainer_email='saito.ayumu1002@icoud.com',
    description='Bridge STDIN/STDOUT pipelines and ROS 2 Twist messages.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stdin_to_twist = twist_stdin_bridge.stdin_to_twist:main',
            'twist_to_stdout = twist_stdin_bridge.twist_to_stdout:main',
        ],
    },
)
