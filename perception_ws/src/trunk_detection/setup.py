from setuptools import setup, find_packages

package_name = 'trunk_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/trunk_detection.launch.py',
            'launch/trunk_detection_viz.launch.py',
            'launch/dataset_collection.launch.py',
            'launch/demo_visualizer.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/detection_params.yaml']),
        ('share/' + package_name + '/models', ['models/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='czb',
    maintainer_email='czb@example.com',
    description='Trunk detection using YOLO algorithm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trunk_detector = trunk_detection.trunk_detector:main',
            'line_fitter = trunk_detection.line_fitter:main',
            'bag_to_images = trunk_detection.bag_to_images:main',
            'dataset_collector = trunk_detection.dataset_collector:main',
            'demo_visualizer = trunk_detection.demo_visualizer:main',
        ],
    },
)
