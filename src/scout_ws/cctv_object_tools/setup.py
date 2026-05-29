from glob import glob
from setuptools import setup

package_name = 'cctv_object_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youngwoo',
    maintainer_email='youngwoo@example.com',
    description='Demo tools for publishing CCTV object PoseArray messages.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz_click_to_pose_array = cctv_object_tools.rviz_click_to_pose_array:main',
        ],
    },
)
