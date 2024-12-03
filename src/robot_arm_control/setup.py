from setuptools import setup, find_packages

package_name = 'robot_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'scservo_sdk'],
    package_dir={
        package_name: package_name,
        'scservo_sdk': 'scservo_sdk',
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jacob',
    maintainer_email='jacob@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = robot_arm_control.servo_driver:main',
        ],
    },
)
