from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    arduinobot_description_directory = get_package_share_directory('arduinobot_description')

    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=os.path.join(arduinobot_description_directory, 'urdf', 'arduinobot.urdf.xacro'),
        description='urdf file'
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(arduinobot_description_directory).parent.resolve())
        ]
    )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    # physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"

    robot_description = ParameterValue(Command([
        'xacro ', 
        LaunchConfiguration('model'),
        " is_ignition:=",
        is_ignition
        ]), 
        value_type=str)

    robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description': robot_description,
                       "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch"
            ), "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", [" -v 4 -r empty.sdf"])
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "arduinobot"
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])