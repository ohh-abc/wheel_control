from setuptools import setup

package_name = 'serial_odom_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/serial_odom_imu_launch.py',
            'launch/ekf_launch.py',
            'launch/combo_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/ekf.yaml',
        ]),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'serial_odom_imu_node = serial_odom_imu.serial_odom_imu_node:main',
        ],
    },
)
