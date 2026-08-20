from setuptools import find_packages, setup

package_name = 'hazard_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abel Dominguez',
    maintainer_email='abel.s.dominguez@gmail.com',
    description='ROS 2 target association, closing-speed, TTC, and fused hazard logic for Prototype_V1',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'target_association_node = hazard_controller.target_association_node:main',
            'fusion_hazard_node = hazard_controller.fusion_hazard_node:main'
        ],
    },
)
