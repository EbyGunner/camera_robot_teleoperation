from setuptools import find_packages, setup

package_name = 'robot_imitation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_imitation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gunner',
    maintainer_email='ebyj23@gmail.com',
    description='Hand-to-robot teleoperation: maps tracked hand states to '
                'MoveIt pose goals and gripper commands for two arms.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # executable name kept for compatibility with existing launch files
            'imitation_algorithm = robot_imitation.imitation_node:main',
        ],
    },
)
