/**
 * @file   mujocoSim_node.cpp
 * @author Anubhav Dogra
 * @data   December 2024
 * @brief  Creates a ROS2 node that interfaces with a MuJoCo simulation.
 */

 #include "mujoco_ros_interface/MujocoInterface.h"
#include <iostream>
class MujocoSimNode : public rclcpp::Node
{
    public:
        MujocoSimNode() : Node("mujoco_interface_node")
        {
            this->declare_parameter<int>("simulation_frequency", 1000);
            this->declare_parameter<int>("visualization_frequency", 20);
            this->declare_parameter<std::string>("xml_path", "");
            this->declare_parameter<std::string>("control_mode", "TORQUE");
            this->declare_parameter<std::string>("publisher_name", "joint_states");
            this->declare_parameter<std::string>("eef_state_publisher_name", "eef_pose");
            this->declare_parameter<std::string>("subscriber_name", "joint_commands");
            this->declare_parameter<std::string>("end_effector_name", "tool_link_ee");
            this->declare_parameter<std::string>("plugin_directory", "opt/mujoco/mujoco-3.2.3/bin");

            this->declare_parameter<double>("proportional_gain", 1.0);
            this->declare_parameter<double>("integral_gain", 0.01);
            this->declare_parameter<double>("derivative_gain", 0.0);

            this->declare_parameter<std::vector<double>>("camera_focal_point", {0.0, 0.0, 0.5});
            this->declare_parameter<double>("camera_distance", 2.5);
            this->declare_parameter<double>("camera_azimuth", 135);
            this->declare_parameter<double>("camera_elevation", -30);
            this->declare_parameter<bool>("camera_orthographic", false);

            // Retrieve parameters
            int simulationFrequency = this->get_parameter("simulation_frequency").as_int();
            int visualizationFrequency = this->get_parameter("visualization_frequency").as_int();
            std::string xmlLocation = this->get_parameter("xml_path").as_string();
            std::string controlMode = this->get_parameter("control_mode").as_string();
            std::string publisherName = this->get_parameter("publisher_name").as_string();
            std::string endEffectorState_publisherName = this->get_parameter("eef_state_publisher_name").as_string();
            std::string subscriberName = this->get_parameter("subscriber_name").as_string();
            std::string endEffectorName = this->get_parameter("end_effector_name").as_string();
            std::string pluginDirectory = this->get_parameter("plugin_directory").as_string();
            
            double proportionalGain = this->get_parameter("proportional_gain").as_double();
            double derivativeGain = this->get_parameter("derivative_gain").as_double();
            double integralGain = this->get_parameter("integral_gain").as_double();
            
            std::vector<double> cameraFocalPoint = this->get_parameter("camera_focal_point").as_double_array();
            double cameraDistance = this->get_parameter("camera_distance").as_double();
            double cameraAzimuth = this->get_parameter("camera_azimuth").as_double();
            double cameraElevation = this->get_parameter("camera_elevation").as_double();
            bool cameraOrthographic = this->get_parameter("camera_orthographic").as_bool();

             // Set the control mode
            ControlMode control_mode;
            if (controlMode == "POSITION")
                control_mode = POSITION;
            else if (controlMode == "VELOCITY")
                control_mode = VELOCITY;
            else if (controlMode == "TORQUE")
                control_mode = TORQUE;
            else
                control_mode = UNKNOWN;

            try
            {
                // Create MujocoInterface object
                auto mujocoSim = std::make_shared<MujocoInterface>
                                                    (xmlLocation,
                                                    publisherName,
                                                    endEffectorState_publisherName,
                                                    subscriberName,
                                                    endEffectorName,
                                                    pluginDirectory,
                                                    control_mode,
                                                    simulationFrequency,
                                                    visualizationFrequency
                                                    );

                // Set feedback gains
                mujocoSim->set_feedback_gains(proportionalGain, integralGain, derivativeGain);

                // Set camera properties
                mujocoSim->set_camera_properties({cameraFocalPoint[0], cameraFocalPoint[1], cameraFocalPoint[2]},
                                                 cameraDistance,
                                                 cameraAzimuth,
                                                 cameraElevation, 
                                                 cameraOrthographic);

                // // Run the simulation indefinitely
                // rclcpp::spin(shared_from_this());
                RCLCPP_INFO(this->get_logger(), "MuJoCo interface initialized successfully.");
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error creating MujocoInterface object: %s", e.what());
                rclcpp::shutdown();
            }
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MujocoSimNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}