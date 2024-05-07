import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import command, LaunchConfiguration


def generate_launch_description():

    xacro_file = os.path.join(get_package_share_directory('robot_desc'),'urdf/robot_description.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output = "screen",
        parameters=[{'robot_description': robot_description}]
    )
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",# affiche le message screen a l'interface graphique
        arguments=["-d",os.path.join(get_package_share_directory("robot_desc"),"rviz","display.rviz")]
    )

    talker_node = Node(
        package="demo_nodes_py",
        executable="talker",
    )
    listener_node = Node(
        package="demo_nodes_py",
        executable="listener"
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])