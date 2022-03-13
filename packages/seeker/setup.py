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
            'capture_node = seeker.capture:main',
            'collision_avoidance_node = seeker.collision_avoidance:main',
            'intake_system_node = seeker.intake_system:main',
            'intel_centroid_node = seeker.intel_centroid:main',
            'robocar_align_node = seeker.align_node:main',
            'state_machine_node = seeker.state_machine:main',
            'simple_states_node = seeker.simple_states:main',
            'webcam_centroid_node = seeker.webcam_centroid:main',
            'webcam_node = seeker.webcam:main'
        ],
    },
)
