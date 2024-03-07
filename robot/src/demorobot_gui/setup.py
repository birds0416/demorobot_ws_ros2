from setuptools import setup
from glob import glob
import os

package_name = 'demorobot_gui'
share_dir = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join(share_dir, 'config'), glob('config/*.yaml')),
        (os.path.join(share_dir, 'scripts'), glob('scripts/*.py')),
        (os.path.join(share_dir, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_main = demorobot_gui.robot_main:main',
            'normal_mode = demorobot_gui.normal:main',
            'emergency_mode = demorobot_gui.emergency:main'
        ],
    },
)
