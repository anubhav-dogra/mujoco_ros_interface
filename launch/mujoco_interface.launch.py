from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
import os

curr_path = os.path.dirname(os.path.abspath(__file__))
# get one step back from the current path
mujoco_ros_interface_path = os.path.abspath(os.path.join(curr_path, os.pardir))

def generate_launch_description():
        xml_path_arg = DeclareLaunchArgument(
                'xml_path',
                default_value='/home/ros/ros2_ws/src/mujoco_ros_interface/test/iiwa14.xml',
                description='Path to MuJoCo xml file'
        )

        # define parameter files to be loaded.
        default_sim_params = ParameterFile(
                "/home/ros/ros2_ws/src/mujoco_ros_interface/config/default_sim.yaml",
                allow_substs=True
        )

        default_camera_params = ParameterFile(
                "/home/ros/ros2_ws/src/mujoco_ros_interface/config/default_camera.yaml",
                allow_substs=True
        )

        default_controller_params = ParameterFile(
                "/home/ros/ros2_ws/src/mujoco_ros_interface/config/default_controller.yaml",
                allow_substs=True
        )

        # define mujoco_ros_interface node
        mujoco_ros_interface_node = Node(
                package='mujoco_ros_interface',
                executable='mujoco_interface_node',
                name='mujoco_interface_node',
                output='screen',
                parameters=[
                        {'xml_path': LaunchConfiguration('xml_path')},
                        default_sim_params,
                        default_camera_params,
                        default_controller_params
                ]
        )

        return LaunchDescription([
                xml_path_arg,
                mujoco_ros_interface_node
        ])