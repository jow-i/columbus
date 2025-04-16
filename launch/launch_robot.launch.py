import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='columbus' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control':'true'}.items()
    )

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','joystick.launch.py'
    #             )])
    # )
    
    # robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
    
    mecanum_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_cont", "--controller-manager", "/controller_manager"],
    )

    delayed_mecanum_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[mecanum_drive_spawner],
        )
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mecanum_drive_spawner,
            on_exit=[joint_broad_spawner],
        )
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
    
    # Launch them all!
    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_mecanum_drive_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        odom_tf_relay_node,
        odom_msg_relay_node
])
