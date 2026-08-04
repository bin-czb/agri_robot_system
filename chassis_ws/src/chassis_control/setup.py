from glob import glob
import os

from setuptools import setup, find_packages

package_name = 'chassis_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', ['config/chassis_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='czb',
    maintainer_email='czb@example.com',
    description='Chassis control for tracked vehicle with RS485 communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_controller = chassis_control.chassis_controller:main',
            'serial_test = chassis_control.serial_test:main',
            'chassis_serial_teleop = chassis_control.serial_teleop:main',
        ],
    },
)
