import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dummy1_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiwon Kim',
    maintainer_email='gjkjw_030@kyonggi.ac.kr',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='molu',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Odomtf = dummy1_bringup.src.odomtf:main'
            'jointstatepub = dummy1_bringup.src.jointstatepub:main'
        ],
    },
)
