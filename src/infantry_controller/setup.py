"""
Infantry Controller 包安装配置

本模块定义 infantry_controller ROS2 包的安装配置。
"""
from setuptools import setup
import os
from glob import glob

package_name = 'infantry_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Infantry Robot Controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_controller = infantry_controller.chassis_controller:main',
            'gimbal_controller = infantry_controller.gimbal_controller:main',
        ],
    },
)
