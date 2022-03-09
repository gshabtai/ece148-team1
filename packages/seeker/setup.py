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
            'webcam_publish_centroid = seeker.webcam_publish_centroid:main',
            'collision_avoidance = seeker.collision_avoidance:main',
            'fan_controller = seeker.fan_controller:main'
        ],
    },
)
