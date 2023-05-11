import os
from glob import glob
from setuptools import setup

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=['path_planner'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
        glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cmiller',
    maintainer_email='curtis.miller@sjsu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner = path_planner.path_planner:main',
            'move = path_planner.move:main',
            'robot = path_planner.robot:main',
            'map = path_planner.map:main',
            'state = path_planner.state:main',
        ],
    },
)
