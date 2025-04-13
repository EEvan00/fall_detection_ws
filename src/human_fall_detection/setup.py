from setuptools import find_packages, setup
from glob import glob

package_name = 'human_fall_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='test',
    maintainer_email='test@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = human_fall_detection.camera_node:main',
            'pose_detection_node = human_fall_detection.pose_detection_node:main',
            'object_detection_node = human_fall_detection.object_detection_node:main',
            'fall_detection_node = human_fall_detection.fall_detection_node:main',
            'visualization_node = human_fall_detection.visualization_node:main',
            'alarm_node = human_fall_detection.alarm_node:main',
        ],
    },

    options={
        'install': {
            'install_scripts': {
                'install_dir': 'lib/' + package_name
            },
        },
        'bdist_wheel': {
            'universal': True
        },
        'build_scripts': {
            'executable': '/usr/bin/env python3'
        },
    },
)
