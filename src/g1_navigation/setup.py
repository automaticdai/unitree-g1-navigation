from glob import glob
from setuptools import find_packages, setup

setup(
    name='g1_navigation', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/g1_navigation']),
        ('share/g1_navigation', ['package.xml']),
        ('share/g1_navigation/launch', glob('launch/*.launch.py')),
        ('share/g1_navigation/config', glob('config/*')),
        ('share/g1_navigation/behavior_trees', glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'],
    entry_points={'console_scripts': [
        'sensor_adapter = g1_navigation.sensor_adapter:main',
        'map_recorder = g1_navigation.map_recorder:main',
        'localizer = g1_navigation.localizer:main',
        'command_bridge = g1_navigation.command_bridge:main',
    ]},
)
