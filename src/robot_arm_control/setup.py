from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['robot_arm_control', 'robot_arm_control.*', 'scservo_sdk', 'scservo_sdk.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your.email@example.com',
    description='Robot arm control package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = robot_arm_control.servo_driver:main',
        ],
    },
)
