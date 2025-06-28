from setuptools import find_packages, setup

package_name = 'unitree_go2_rplidar_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_launch.py']),
        ('share/' + package_name + '/config', ['config/slam.yaml']),
        ('share/' + package_name + '/urdf', ['urdf/unitree_go2.urdf.xacro', 'urdf/unitree_go2.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jan',
    maintainer_email='TangmereCottage@protonmail.com',
    description='Unitree Go2 robot SLAM package with RPLidar',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_to_tf = src.pose_to_tf:main',
        ],
    },
)
