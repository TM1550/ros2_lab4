from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_multi_target'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_target.launch.py']),
        ('share/' + package_name + '/resource', ['resource/multi_target.rviz']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Turtle multi target following',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf2_broadcaster = turtle_multi_target.turtle_tf2_broadcaster:main',
            'target_switcher = turtle_multi_target.target_switcher:main',
            'turtle_controller = turtle_multi_target.turtle_controller:main',
            'keyboard_switch = turtle_multi_target.keyboard_switch:main',
        ],
    },
)