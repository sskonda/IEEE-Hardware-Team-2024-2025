from setuptools import find_packages, setup

package_name = 'py_robot'

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
    maintainer='kapow',
    maintainer_email='iyer.h.nikhil@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'differential_drive = py_robot.differential_drive:main',
            'mpu6500 = py_robot.mpu6500:main',
            'tag_pose_publisher = py_robot.tag_pose_publisher:main',
            'turtle_pose_to_odom = py_robot.turtle_pose_to_odom:main',
            'hardware_interface = py_robot.hardware_interface:main',
        ],
    },
)  