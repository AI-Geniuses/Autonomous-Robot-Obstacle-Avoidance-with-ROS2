import os
from os import pathsep
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('robot_desc'),'urdf/robot_description.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_prefix = get_package_prefix('robot_desc')
    model_path = os.path.join(get_package_share_directory('robot_desc'),'models')
    model_path+= pathsep + os.path.join(robot_prefix,'share')

    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH",model_path)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output = "screen",
        parameters=[{'robot_description': robot_description,
        'use_sim_time': True}]
    )

    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"),"launch","gzserver.launch.py"
    )))

    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"),"launch","gzclient.launch.py"
    )))

    # spawn_entity = Node(
    #     package="gazebo_ros",
    #     executable="spawn_entity.py",
    #     arguments=["-entity","two_wheel_robot","-topic","robot_description"],
    #     output = "screen",
    # )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'two_wheel_robot'],
                    output='screen')
    


    return LaunchDescription([
        robot_state_publisher,
        env_variable,
        start_gazebo_server,
        start_gazebo_client,
        spawn_entity
    ])