from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_to_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rss',
    maintainer_email='eb4626s@ad.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_to_ros = yolo_to_ros.yolo_to_ross:main',
            'yolo_to_ros_comp = yolo_to_ros.yolo_to_ros_comp:main',
            'hand_sign = yolo_to_ros.hand_sign:main',
            'det = yolo_to_ros.detection_node:main',
            'room = yolo_to_ros.room_announcer_node:main',
        ],
    },
)
