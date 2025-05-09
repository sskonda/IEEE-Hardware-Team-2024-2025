from setuptools import find_packages, setup

package_name = 'robot_control'

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
    maintainer='katherine',
    maintainer_email='katherine@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_to_pose = robot_control.drive_to_pose:main',
            'drive_to_path = robot_control.drive_to_path:main',
            'autonomous = robot_control.autonomous:main',
            'sensor_fusion = robot_control.fusion:main',
        ],
    },
)
