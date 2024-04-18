from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'snchallenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bennie',
    maintainer_email='s3835358@student.rmit.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"best_effort_repeater = {package_name}.best_effort_repeater:main",
            f"publish_hazard = {package_name}.publish_hazard:main",
            f"publish_navpath = {package_name}.publish_navpath:main"
        ],
    },
)