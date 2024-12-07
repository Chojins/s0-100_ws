from setuptools import setup, find_namespace_packages
import os
from glob import glob

print("Running setup.py")
package_name = 'robot_arm_control'

# Debug: Print current directory and found files
print(f"Current directory: {os.getcwd()}")
launch_files = glob('launch/*.launch.py')
print(f"Found launch files: {launch_files}")

setup(
    name=package_name,
    version='0.0.0',
    packages=find_namespace_packages(include=['robot_arm_control*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/meshes', glob('meshes/*.STL')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name, ['servo_hardware.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
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
