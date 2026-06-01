from glob import glob
from setuptools import find_packages, setup

package_name = 'perception_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob('resource/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seoyunseo',
    maintainer_email='yunseo0812@naver.com',
    description='Perception package',
    license='TODO',
    entry_points={
        'console_scripts': [
           'cam_to_map_node = perception_pkg.cam_to_map_node:main',
            'cctv_fov_node = perception_pkg.cctv_fov_node:main',
            'cctv_pose_node = perception_pkg.cctv_pose_node:main',
            'cctv_to_map_node = perception_pkg.cctv_to_map_node:main',
            'fusion_node = perception_pkg.fusion_node:main',
            'lidar_object_node = perception_pkg.lidar_object_node:main',
            'pixel_picker_node = perception_pkg.pixel_picker_node:main',
            'yolo_detector_node = perception_pkg.yolo_detector_node:main',
        ],
    },
)
