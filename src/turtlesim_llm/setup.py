from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlesim_llm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/whisper_params.yaml']),  
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
            # 'turtlesim_llm.turtlesim_llm_controller = turtlesim_llm.turtlesim_llm_controller:main',
            'llm_controller = turtlesim_llm.turtlesim_llm_controller:main',
            'voice_to_command = turtlesim_llm.voice_to_command:main',
            'create3_llm_controller = turtlesim_llm.create3_llm_controller:main'

        ],
    },
)
