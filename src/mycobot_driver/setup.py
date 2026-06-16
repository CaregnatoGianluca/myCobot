import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mycobot_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # The Pi-side bridge is installed too, so it can be located and scp'd from here.
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gianluca Caregnato',
    maintainer_email='gianluca.caregnato@gmail.com',
    description='Real-robot driver bridging MoveIt2 to the myCobot 280 Pi over TCP.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mycobot_driver_node = mycobot_driver.mycobot_driver_node:main',
        ],
    },
)
