from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'um982_ntrip'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='czb',
    maintainer_email='czb@todo.todo',
    description=(
        'ROS 2 NTRIP client and NavSatFix publisher for the UM982 GNSS '
        'receiver.'),
    license='MIT',
    entry_points={
        'console_scripts': [
            'um982_ntrip_node = um982_ntrip.node:main',
            'rtk_subscriber = um982_ntrip.rtk_subscriber:main',
        ],
    },
)
