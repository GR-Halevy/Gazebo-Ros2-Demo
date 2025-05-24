from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_package')

    robot1_file = os.path.join(pkg_share, 'urdf', 'robot1.urdf')
    robot2_file = os.path.join(pkg_share, 'urdf', 'robot2.urdf')



    slam_launch_file = os.path.join(pkg_share, 'launch', 'online_async_launch1.py')
    #slam_launch_file2 = os.path.join(pkg_share, 'launch', 'online_async_launch2.py')


    slam_launch1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'map_topic': '/map1',
        }.items()
    )
    # slam_launch2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(slam_launch_file2),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #         'map_topic': '/map2',
    #     }.items()
    # )


    # slam_process1 = ExecuteProcess(
    #     cmd=['ros2', 'launch', 'my_robot_package', 'online_async_launch1.py', 'use_sim_time:=true'],
    # )
    slam_process2 = ExecuteProcess(
        cmd=['ros2', 'launch', 'my_robot_package', 'online_async_launch2.py', 'use_sim_time:=true', 'map_topic:=/map2'],
    )

    world_file = os.path.join(pkg_share, 'worlds', 'harmonic.sdf')
    # world_file2 = os.path.join(pkg_share, 'worlds', 'building_robot2.sdf')
    gz_process = ExecuteProcess(
        cmd=['gz', 'sim', world_file], # add -v 4 for verbose
        output='screen',
        shell=True,
        name='gazebo_sim',
    )

    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_process,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )
    return LaunchDescription([
        gz_process,
        shutdown_handler,
        # Bridge, ] ros2 to gazebo, [ gazebo to ros2, @ both ways
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # drive robots
                '/model/X1_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/X1_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

                # pass odometry and pose to ros2 from gazebo
                '/model/X1_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/X1_1/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
                '/model/X1_2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/X1_2/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
                
                # lidar and clock, clock is needed for simulation time and the current nodes require it
                '/lidar1@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/lidar2@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', 
                
                # joint states for both robots for rviz visualization of joints, joint info is being published from gazebo with
                # joint_state publisher plugin
                '/world/harmonic/model/X1_1/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/world/harmonic/model/X1_2/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                
                # remapping to expected topics
                '--ros-args',                   # only 1 --ros-args required/allowed
                '--remap', '/lidar1:=/scan1', '--remap', '/lidar2:=/scan2',

                # per default robot_state_publisher expectes /joint_states topic, but we remap below anyway so can name it how you like
                '--remap', '/world/harmonic/model/X1_1/joint_state:=/X1_1/joint_states', 
                '--remap', '/world/harmonic/model/X1_2/joint_state:=/X1_2/joint_states',

                ],
            output='screen',
        ),
        # custom node publishing tf's for both robots, required for rviz2
        Node(
            package='my_robot_package',
            executable='tf_publisher',
            name='tf_publisher',
            output='screen',
            parameters=[
                {'use_sim_time' : True},
            ]
        ),
        # publishes joint state data for rviz2
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='X1_1',
            parameters=[{'use_sim_time': True}],
            arguments=[robot1_file],
            remappings=[
                ('/joint_states', '/X1_1/joint_states')
            ],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='X1_2',
            parameters=[{'use_sim_time': True}],
            arguments=[robot2_file],
            remappings=[
                ('/joint_states', '/X1_2/joint_states')
            ],
            output='screen'
        ),
        # static transform, to complete the tf tree
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf_X1_1',
            arguments=['0', '0', '0','0', '0', '0', 'X1_1_base_link', 'X1_1/X1_1_base_link/gpu_lidar'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf_X1_2',
            arguments=['0', '0', '0', '0', '0', '0', 'X1_2_base_link', 'X1_2/X1_2_base_link/gpu_lidar'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        # both slam toolbox processes
        # one thing here to note, I couldnt get them both to run via the launch file so one is ran via ExecuteProcess instead
        slam_launch1,
        slam_process2,
        Node(
            package='my_robot_package',
            executable='path_follower',
            name='path_follower',
            output='screen',
            parameters=[
                {'use_sim_time' : True},
                {'robot_name': 'X1_1'}
            ]
        ),
        Node(
            package='my_robot_package',
            executable='path_follower',
            name='path_follower_2',
            output='screen',
            parameters=[
                {'use_sim_time' : True},
                {'robot_name': 'X1_2'}
            ]
        ),
        # static transform from map1 to merge map, this is so rviz2 knows where to show merged_map
        # This is because there is no merge_map frame as of now, instead we just use map1's frame with this transform
        # it should be fine since merged_map is just map1 with map2 added so its frame should align with that of map1
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='merge_map_map1_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'merge_map', 'map1'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        # static transform from map1 to map2, this is so merged_map can also be visualized using map2's frame 
        # via this transform. map2->map1->merged_map
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map1_map2_static_tf',
            arguments=['5.3', '5.6', '0', '0', '0', '0', 'map1', 'map2'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='my_robot_package',
            executable='map_merger',
            name='map_merger',
            output='screen',
            parameters=[
                {'use_sim_time' : True},
            ]
        )
    ])
