import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'minesweeper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fjourda',
    maintainer_email='fjourda@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                    'metalDetector = minesweeper.metalDetector:main',
                    'shore_follower_drive_node = shore_follower_drive_base.shore_follower_drive:main',
                    'shore_follower_drive_reg = shore_follower_drive_base_reg.shore_follower_drive_reg:main',
            ],
    },
)
