import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Get the path to the mujoco_interface package
    mujoco_interface_dir = get_package_share_directory('mujoco_ros_interface')

    # Load config files
    config_dir      =    os.path.join(mujoco_interface_dir, 'config')
    camera_params   =    os.path.join(config_dir, 'default_camera.yaml')
    sim_params      =    os.path.join(config_dir, 'default_sim.yaml')
    control_params  =    os.path.join(config_dir, 'default_controller.yaml')

    # Define the LaunchConfiguration for xml_path using the path inside mujoco_interface
    xml = LaunchConfiguration('xml', default=os.path.join(mujoco_interface_dir, 'test/scene.xml'))

    return LaunchDescription([
        # Node configuration
        Node
        (
            package    = 'mujoco_ros_interface',
            executable = 'mujoco_interface_node',
            output     = 'screen',
            parameters =
            [
                {'xml': xml},
                camera_params,
                sim_params,
                control_params
                
            ]
        )
    ])