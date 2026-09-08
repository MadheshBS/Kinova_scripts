from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'kinova_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='madhesh',
    maintainer_email='madhesh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_goal_demo = kinova_scripts.pose_goal_demo:main',
            'pose_gripper_demo = kinova_scripts.pose_gripper_demo:main',
            'grasp_isolation_test = kinova_scripts.grasp_isolation_test:main',
            'spawn_cube_collision = kinova_scripts.spawn_cube_collision:main',
        ],
    },
)
