from setuptools import setup
from glob import glob
package_name = 'vel_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'PID'],
    package_dir = {'PID': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (glob('launch/*.launch.py'))
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
            'vel_controller = vel_controller.vel_controller:main'
        ],
    },
)
