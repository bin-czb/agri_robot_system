from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'agri_map_integration'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*'),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz'),
        ),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='czb',
    maintainer_email='czb@example.com',
    description=(
        'Georeferenced orthophoto, RTK and VSLAM visualization integration.'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orthophoto_publisher = '
            'agri_map_integration.orthophoto_publisher:main',
            'tree_marker_publisher = '
            'agri_map_integration.tree_marker_publisher:main',
            'vslam_map_aligner = '
            'agri_map_integration.vslam_map_aligner:main',
            'occupancy_grid_overlay = '
            'agri_map_integration.occupancy_grid_overlay:main',
        ],
    },
)
