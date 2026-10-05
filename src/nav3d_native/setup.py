from glob import glob
from setuptools import find_packages, setup

package_name = 'nav3d_native'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Chen Lvping',
    maintainer_email='chenlvping@example.com',
    description='Native ROS 2 navigation stack with deterministic simulation validation.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'simulator = nav3d_native.simulator_node:main',
        'mapper = nav3d_native.mapping_node:main',
        'localizer = nav3d_native.localization_node:main',
        'planner = nav3d_native.planner_node:main',
        'controller = nav3d_native.controller_node:main',
        'demo_supervisor = nav3d_native.demo_supervisor:main',
    ]},
)
