from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wa_birrt_star'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'),  # ← add this
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='maxyk@umd.edu',
    description='WA*-Bi-RRT* planner for ENPM661',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node        = wa_birrt_star.planner_node:main',
            'path_follower       = wa_birrt_star.path_follower_node:main',
            'static_obstacle_pub = wa_birrt_star.static_obstacle_publisher:main',
            'static_only_pub     = wa_birrt_star.static_only_publisher:main',  # ← add
        ],
    },
)