import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_description'

    # 1. Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(pkg_name),'launch','rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Gazebo Simulation
    world_file = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'obstacle_world.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 3. Spawn Robot Entity
    spawn_robot = Node(package='ros_gz_sim', executable='create', arguments=['-topic', 'robot_description', '-name', 'my_robot'], output='screen')

    # 4. ROS-GZ Bridge Configuration
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/obs_1/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/obs_2/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/obs_3/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist'
        ],
        output='screen'
    )

    return LaunchDescription([rsp, gazebo, spawn_robot, bridge])