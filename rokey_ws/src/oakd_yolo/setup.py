from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'oakd_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # launch 폴더 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # config 폴더가 있다면
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyeonhee',
    maintainer_email='hyeonhee@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_depth_oakd = oakd_yolo.yolo_depth_oakd:main',
            'camera_test = oakd_yolo.camera_test:main',
	    'mapping = oakd_yolo.mapping:main',

        ],
    },
)
