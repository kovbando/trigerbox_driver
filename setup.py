from setuptools import setup
from glob import glob
import os

package_name = 'triggerbox'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the launch directory and its files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bandó Kovács, ELTE-IK',
    maintainer_email='kovbando@inf.elte.hu',
    description='A ROS2 package for publishing trigger IDs via UDP.',
    license='GLWTPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'triggerid_publisher = triggerbox.triggerid_publisher:main',
        ],
    },
)
