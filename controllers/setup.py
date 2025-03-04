import glob
import os
from setuptools import find_packages
from setuptools import setup

package_name = 'controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/controllers/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
        ('share/controllers/launch', glob.glob(os.path.join('launch', '*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='pliam1105@gmail.com',
    description='Controllers for RSS 2025 racecar',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_controller = controllers.safety_controller:main',
            'keyboard_controller = controllers.keyboard_controller:main',
        ],
    },
)
