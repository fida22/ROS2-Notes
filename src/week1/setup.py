from setuptools import find_packages, setup
import os

package_name = 'week1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),[os.path.join('launch',f)for f in os.listdir('launch') if f.endswith('.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fida',
    maintainer_email='fida@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "publisher_node=week1.publisher_node:main",
            "subscriber_node=week1.subscriber_node:main",
        ],
    },
)
