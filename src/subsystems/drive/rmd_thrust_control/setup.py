import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'rmd_thrust_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adrian Cires',
    maintainer_email='loop@adriancires.com',
    description='Temporary RMD Motor Control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tm_node = rmd_thrust_control.thrustmaster_node:main',
            'tm_control_node = rmd_thrust_control.tm_control_node:main',
            'rover_node = rmd_thrust_control.rover_node:main',
            'motor_node = rmd_thrust_control.motor_node:main',
            'twist2joint_node = rmd_thrust_control.AckermannNode:main'
        ],
    },
)
