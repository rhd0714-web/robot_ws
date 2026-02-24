from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ah_bot_dashboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ↓ 이 줄이 없으면 launch 파일을 못 찾음
        ('share/' + package_name + '/launch',
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhd0714',
    maintainer_email='rhd0714@todo.todo',
    description='AH Bot Dashboard Package',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'dashboard   = ah_bot_dashboard.dashboard:main',
            'controller  = ah_bot_dashboard.controller_node:main',
            'vision_node = ah_bot_dashboard.vision_node:main',
        ],
    },
)
