from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'butler_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files from package root
        (os.path.join('share', package_name, 'launch'),
            glob('*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arunesh',
    maintainer_email='arunesh@todo.todo',
    description='Butler robot package for TurtleBot3 simulation',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # butler_node.py lives at butler_robot/butler_node.py
            'butler_node = butler_robot.butler_node:main',
            # order_publisher.py lives at butler_robot/order_publisher.py
            'order_publisher = butler_robot.order_publisher:main',
        ],
    },
)
