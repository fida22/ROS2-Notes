from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'four_wheel_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch','*.launch.py'))),
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'),
         glob(os.path.join('urdf','*.*'))),
        # Install world files
        (os.path.join('share', package_name, 'worlds'),
         glob(os.path.join('worlds','*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fida',
    maintainer_email='fidasaifudheen05@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_stop = four_wheel_robot.obstacle_stop_node:main',
        ],
    },
)
