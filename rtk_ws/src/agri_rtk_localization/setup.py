from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'agri_rtk_localization'


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
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    scripts=[
        'scripts/configure_ntrip_credentials',
        'scripts/start_um982_ntrip',
    ],
    zip_safe=True,
    maintainer='czb',
    maintainer_email='czb@example.com',
    description='RTK global localization adapter and RViz trajectory output',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtk_localizer = agri_rtk_localization.rtk_localizer:main',
        ],
    },
)
