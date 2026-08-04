from setuptools import setup, find_packages

package_name = 'navigation_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/navigation_controller.launch.py',
            'launch/navigation_controller_depth.launch.py',
            'launch/navigation_row_offset_stop.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/navigation_params.yaml',
            'config/navigation_row_offset_stop.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='czb',
    maintainer_email='czb@example.com',
    description='Navigation and tracking controller for trunk following',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_controller = navigation_controller.navigation_controller:main',
            'navigation_controller_depth = navigation_controller.navigation_controller_depth:main',
        ],
    },
)

