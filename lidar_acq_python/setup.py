from setuptools import setup
from setuptools import find_packages

package_name = 'lidar_acq_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'laser_geometry'],
    package_dir = {'lidar_acq_python': './lidar_acq_python',
                    'laser_geometry': '../deepracer_deps/laser_geometry/src/laser_geometry'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taylor',
    maintainer_email='taylormaurer4323@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_acq = lidar_acq_python.lidar_acq:main',
            
        ],
    },
)
