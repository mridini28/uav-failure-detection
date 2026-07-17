from setuptools import find_packages, setup

package_name = 'uav_monitor'

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
    maintainer='mitta',
    maintainer_email='mitta@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'telemetry_logger = uav_monitor.telemetry_logger:main',
        'anomaly_detector = uav_monitor.anomaly_detector:main',
        'fault_injector = uav_monitor.fault_injector:main',
        'response_node = uav_monitor.response_node:main',
        'data_collector = uav_monitor.data_collector:main',
        'ml_anomaly_detector = uav_monitor.ml_anomaly_detector:main',
    ],
    },
)
