from setuptools import setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_launch_file.launch.py']),
        ('share/' + package_name + '/launch', ['launch/online_async_launch1.py']),
        ('share/' + package_name + '/launch', ['launch/online_async_launch2.py']),
        ('share/' + package_name + '/config', ['config/mapper_params_online_async1.yaml']),
        ('share/' + package_name + '/config', ['config/mapper_params_online_async2.yaml']),

        
        ('share/' + package_name + '/meshes', ['meshes/chassis.dae', 'meshes/wheel.dae']),
        ('share/' + package_name + '/urdf', ['urdf/robot1.urdf', 'urdf/robot2.urdf']),
        ('share/' + package_name + '/worlds', ['worlds/harmonic.sdf', 'worlds/building_robot2.sdf']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Basic robot path-following package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'path_follower = my_robot_package.path_follower:main',
            'tf_publisher = my_robot_package.tf_publisher:main',
            'map_merger = my_robot_package.map_merger:main',
        ],
    },
)
