from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ackermann_iot_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tinvo',
    maintainer_email='tinvo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'system_health_node = ackermann_iot_bridge.system_health_node:main',
            'telemetry_aggregator_node = ackermann_iot_bridge.telemetry_aggregator_node:main',
            'transport_manager_node = ackermann_iot_bridge.transport_manager_node:main',
        ],
    },
)
