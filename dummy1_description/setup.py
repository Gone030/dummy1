import os
from glob import glob
from struct import pack
from setuptools import find_packages
from setuptools import setup

package_name = 'dummy1_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf','*.urdf.xacro'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz','*.rviz')),)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiwon Kim',
    maintainer_email='gjkjw_030@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
