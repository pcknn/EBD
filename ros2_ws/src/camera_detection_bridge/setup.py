from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'camera_detection_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abel Dominguez',
    maintainer_email='abel.s.dominguez@gmail.com',
    description='ROS 2 camera and detection bridge for the Prototype_V1 perception pipeline',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'zmq_detection_bridge = camera_detection_bridge.zmq_detection_bridge:main',
            'tcp_image_bridge = camera_detection_bridge.tcp_image_bridge:main',
            'detection_annotation_bridge = camera_detection_bridge.detection_annotation_bridge:main',
        ],
    },
)
