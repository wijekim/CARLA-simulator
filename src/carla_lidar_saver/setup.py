from setuptools import find_packages, setup

package_name = 'carla_lidar_saver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airlab',
    maintainer_email='airlab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'saver = carla_lidar_saver.carla_lidar_saver:main',
        'viz   = carla_lidar_saver.viz_ply:main',
        ],
    },
)
