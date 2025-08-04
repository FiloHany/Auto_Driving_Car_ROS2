from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,LaunchConfiguration

def generate_launch_description():
    #contain the name of the directory of the urdf model that we want to visualize  
    model_arg = DeclareLaunchArgument(
        name="model",
        # this function will automatically look for the bumperbot description ros2 package and it wiil get it full directory path  
        default_value= os.path.join(get_package_share_directory("bumperbot_description"),"urdf","bumperbot.udrf.xacro"),
        description="Absolute path to robot URDF file"
    )
    # contain path of a URDF model & Command to change exstantion frpm xacro to URDF model to Play it that can use in visualization & rviz
    #LaunchConfiguration to read the Declare one 
    robot_description = ParameterValue(Command(["xacro ",LaunchConfiguration("model")]), value_type=str)

    # create a node that will be publish
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    rviz_node = Node(
       package="rviz2",
       executable="rviz2",
       name="rviz2",
       output="screen",
       #-d to choose a configration that we want to visualize
       arguments=["-d",os.path.join(get_package_share_directory("bumperbot_description"),"rviz","display.rviz")]

    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])