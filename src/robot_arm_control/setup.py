from setuptools import setup
import os
from glob import glob

package_name = 'robot_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join(package_name, 'launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join(package_name, 'urdf', '*'))),
        (os.path.join('share', package_name, 'meshes'),
            glob(os.path.join(package_name, 'meshes', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Robot arm control package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = robot_arm_control.servo_driver:main',
        ],
    },
)
