from setuptools import setup
from glob import glob
import os

package_name = 'seeker'

setup(
    name = package_name,
    version = '0.0.0',
    packages = [package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
            (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_centroid = seeker.webcam_centroid:main',
            'collision_avoidance_node = seeker.collision_avoidance:main',
            'intake_system_node = seeker.intake_system:main',
            'state_machine_node = seeker.state_machine:main',
            'simple_states_node = seeker.simple_states:main',
            'capture_node_node = seeker.capture_node:main'
        ],
    },
)
