from glob import glob
from setuptools import find_packages, setup


package_name = 'g1_nav_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='G1 team',
    maintainer_email='robot@example.com',
    description='Safe Nav2 cmd_vel to Unitree G1 sport API bridge.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'cmdvel_to_sport = g1_nav_bridge.cmdvel_to_sport:main',
    ]},
)
