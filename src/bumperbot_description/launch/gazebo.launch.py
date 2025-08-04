from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable,IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
    #Create an environment
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH",os.path.join(get_package_prefix("bumperbot_description"),"share"))

    #in Ros2 we can nested a two launch file 
    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("gazebo_ros"),"launch","gzserver.launch.py")
    ))

    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("gazebo_ros"),"launch","gzclient.launch.py")
    ))

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity","bumperbot","-topic","robot_description"],
        output="screen"
    )
    
    return LaunchDescription([
        model_arg,
        env_var,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
    ])