from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_nav2_gz_garden'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ]+ [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('urdf') for file in files
    ]+
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('meshes') for file in files
    ]+
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('models') for file in files
    ]+
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('worlds') for file in files
    ]
    +
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('plugins') for file in files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Manuel Ledesma Rangel',
    maintainer_email='jujuan27@hotmail.com',
    description='Tecnologico de Monterrey 8th semester project for the Robotics and Digital Systems Engineering major. This project consists in using ROS2 Navigation Stack (Nav2) for autonomous navigation of a puzzlebot robot in a student-designed maze.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
