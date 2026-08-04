from glob import glob
from setuptools import find_packages, setup


package_name = 'agri_global_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='czb',
    maintainer_email='czb@example.com',
    description='AOA global pose adapter and robust navigation-side filter',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aoa_pose_filter = agri_global_localization.aoa_pose_filter:main',
            'aoa_map_transform = '
            'agri_global_localization.aoa_map_transform:main',
            'aoa_bag_replay = agri_global_localization.bag_replay:main',
        ],
    },
)
