from setuptools import find_packages, setup
import os

package_name = 'jank_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/jank_drive.launch.py']),
    ],
    install_requires=['setuptools', 'pygame>=2.0.0'],
    zip_safe=True,
    maintainer='umdloop',
    maintainer_email='ciresadrian@hotmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jank_joystick = jank_drive.jank_joystick:main',
            'jank_drive = jank_drive.jank_drive:main',
        ],
    },
)
