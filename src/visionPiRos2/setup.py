from setuptools import setup
import os
from glob import glob

package_name = 'visionPiRos2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='Vision Pi ROS2 package for computer vision',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'object_detection_node = visionPiRos2.object_detection_node:main',
            'gui = visionPiRos2.gui:main'
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],

)

