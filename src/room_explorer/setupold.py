from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'room_explorer'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        include=[package_name, package_name + '.*']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*.yaml')
        ),
    ],

    install_requires=[
        'setuptools',
        'PyYAML'
    ],

    zip_safe=True,

    description='Room exploration and classification for the Robot Challenge',

    license='Apache License 2.0',

    entry_points={
        'console_scripts': [
            'room_explorer_node = room_explorer.room_explorer_node:main',
        ],
    },
)