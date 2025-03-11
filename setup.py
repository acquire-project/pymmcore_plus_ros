from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pymmcore_plus_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Justin Eskesen',
    maintainer_email='jeskesen@chanzuckerberg.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pymmcore_plus_node = pymmcore_plus_ros.pymmcore_plus_node:main'
        ],
    },
)
