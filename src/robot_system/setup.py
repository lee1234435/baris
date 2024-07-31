from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/ui', glob('ui/*.ui')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xyzcorp',
    maintainer_email='martin@xyzcorp.io',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'RobotSystemNode = robot_system.RobotSystemNode:main',
            'DispenserNode = robot_system.DispenserNode:main',
            'RobotControllerNode = robot_system.RobotControllerNode:main'
        ],

    },
)
