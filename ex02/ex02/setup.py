from setuptools import find_packages, setup

package_name = 'ex02'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtlesim_carrot.launch.py']),
        ('share/' + package_name + '/resource', ['resource/carrot.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Turtle TF2 with carrot frame',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf2_broadcaster = ex02.turtle_tf2_broadcaster:main',
            'turtle_tf2_listener = ex02.turtle_tf2_listener:main',
            'carrot_broadcaster = ex02.carrot_broadcaster:main',
        ],
    },
)