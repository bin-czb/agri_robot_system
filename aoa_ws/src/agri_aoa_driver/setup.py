from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'agri_aoa_driver'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='czb',
    maintainer_email='czb@todo.todo',
    description='ROS 2 driver package for the ground AOA rig and UAV pose source.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aoa_localization_node = '
            'agri_aoa_driver.aoa_localization_node:main',
            'uav_gps_node = agri_aoa_driver.uav_gps_node:main',
            'aoa_debug_viewer = agri_aoa_driver.aoa_debug_viewer:main',
        ],
    },
)
