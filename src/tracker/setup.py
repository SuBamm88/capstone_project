import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youngwoo',
    maintainer_email='youngwoo6770@naver.com',
    description='CCTV 객체 Kalman + Hungarian 추적 및 예측 궤적 생성 (capstone)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tracker_node = tracker.tracker_node:main',
        ],
    },
)
