from setuptools import find_packages, setup

package_name = 'rokey_pjt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'rclpy',
                      'geometry_msgs',
                      'sensor_msgs',
                      'tf2_ros',
                      'tf2_geometry_msgs'],
    zip_safe=True,
    maintainer='hyeonhee',
    maintainer_email='hyeonhee@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge2 = rokey_pjt.robot2_mqtt_bridge:main',
            'bridge3 = rokey_pjt.robot3_mqtt_bridge:main',
            'nav2 = rokey_pjt.nav_to_pose:main',
            'nav3 = rokey_pjt.nav_to_pose_topic:main'
        ],
    },
)
