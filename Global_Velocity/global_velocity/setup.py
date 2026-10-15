from setuptools import find_packages, setup

package_name = 'global_velocity'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GolamP',
    maintainer_email='golamprasetyo99@gmail.com',
    description='ROS 2 node to calculate and display global robot velocity.',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'global_velocity_node = global_velocity.global_velocity_node:main',
        ],
    },
)