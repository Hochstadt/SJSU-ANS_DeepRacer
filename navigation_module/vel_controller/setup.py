from setuptools import setup
from glob import glob
import os
package_name = 'vel_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'PID'],
    package_dir = {'PID': '.'},
    data_files=[
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
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
