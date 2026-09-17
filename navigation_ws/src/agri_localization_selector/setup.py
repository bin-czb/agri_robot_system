from glob import glob
from setuptools import setup

setup(
    name='agri_localization_selector', version='0.1.0',
    packages=['agri_localization_selector'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/agri_localization_selector']),
        ('share/agri_localization_selector', ['package.xml', 'README.md']),
        ('share/agri_localization_selector/config', glob('config/*.yaml')),
        ('share/agri_localization_selector/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='czb', maintainer_email='2843896617@qq.com', license='MIT',
    description='Integrity-gated exclusive RTK/AOA measurement selection',
    entry_points={'console_scripts': [
        'localization_selector = agri_localization_selector.node:main',
    ]},
)
