import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate your package directory (Change 'your_package_name' to your actual package)
    pkg_share = get_package_share_directory('crazyflie_description')
    
    # 2. Define path to your URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'crazy_fly.urdf')
    world_path=os.path.join(pkg_share, 'world', 'new_world.world') # Optional: If you have a custom world file
    # 3. Read the URDF file contents
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 4. Robot State Publisher Node (Broadcasts URDF to /robot_description topic)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 5. Include the Gazebo Sim launch file (Standard ROS 2 Jazzy / Harmonic syntax)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items() # Open custom world running (-r)
    )

    # 6. Spawn the robot entity using the /robot_description topic
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_custom_robot',
            '-z', '0.5' # Spawn slightly off the ground safely
        ],
        output='screen'
    )
    # 6. Bridge Node (Pipes topics like /odom and /cmd_vel between ROS 2 and Gazebo)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
'/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
          '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
          
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge
    ])