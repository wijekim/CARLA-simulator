from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'carla_path_visualizer'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='CARLA path and robot position visualizer using matplotlib',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_visualizer = carla_path_visualizer.path_visualizer_node:main',
        ],
    },
)
