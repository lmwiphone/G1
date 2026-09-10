from glob import glob
from setuptools import setup
setup(name='g1_nav_bridge', version='0.1.0', packages=['g1_nav_bridge'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/g1_nav_bridge']),
                  ('share/g1_nav_bridge', ['package.xml']),
                  ('share/g1_nav_bridge/launch', glob('launch/*.launch.py')),
                  ('share/g1_nav_bridge/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='G1 navigation project', maintainer_email='maintainer@example.com',
      description='G1 Lightning integration', license='Apache-2.0',
      entry_points={'console_scripts': [
          'pose_bridge=g1_nav_bridge.pose_bridge:main',
          'unitree_joint_state_bridge=g1_nav_bridge.unitree_bridge:joint_main',
          'unitree_command_bridge=g1_nav_bridge.unitree_bridge:command_main']})
