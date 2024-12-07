from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rohand_urdf_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
        (os.path.join('share', package_name, 'meshes_l'), glob('meshes_l/**')),
        (os.path.join('share', package_name, 'meshes_r'), glob('meshes_r/**')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/**'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oymotion',
    maintainer_email='hebin7611@hotmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rohand_urdf_node = scripts.rohand_urdf:main',
            'rohand_joint_state_gui = scripts.rohand_joint_state_gui:main',
        ],
    },
)
