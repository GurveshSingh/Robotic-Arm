from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def generate_launch_description():

    kinematics_yaml = yaml.safe_load(
        open(os.path.join(
            get_package_share_directory("my_robot_moveit_config"),
            "config", "kinematics.yaml"
        ))
    )

    return LaunchDescription([
        Node(
            package="my_robot_commander",
            executable="commander",
            name="commander",
            output="screen",
            parameters=[
                {"robot_description_kinematics": kinematics_yaml}
            ]
        )
    ])