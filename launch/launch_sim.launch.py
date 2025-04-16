import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='columbus' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'extra gazebo args': '--ros-args --params-file' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'columbus'],
                        output='screen')


    mecanum_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )


    odom_tf_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='odom_tf_relay',
        output='screen',
        parameters=[{
            'input_topic': '/mecanum_cont/tf_odometry', # Source topic
            'output_topic': '/tf'                            # Destination topic
        }],
        # You might need to explicitly set the message type if relay struggles to auto-detect
        # arguments=['--ros-args', '-p', 'topic_type:=tf2_msgs/msg/TFMessage']
    )
   
    

    odom_msg_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='odom_msg_relay',
        output='screen',
        parameters=[{
            'input_topic': '/mecanum_cont/odometry', # Source Odometry topic
            'output_topic': '/odom'                        # Destination standard topic
        }],
        # Message type is usually auto-detected, but can be specified if needed:
        # arguments=['--ros-args', '-p', 'topic_type:=nav_msgs/msg/Odometry']
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        mecanum_drive_spawner,
        joint_broad_spawner,
        odom_tf_relay_node,
        odom_msg_relay_node
    ])