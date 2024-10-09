/**
 * @file   mujocoSim_node.cpp
 * @author Jon Woolfrey
 * @data   August 2024
 * @brief  Creates a ROS1 node that interfaces with a MuJoCo simulation.
 */

#include <mujoco_ros_interface/MujocoInterface.h>
#include <iostream>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mujoco_interface_node");  // Starts up ROS1
    
    ////////////////////////////////////////////////////////////////////////////////////////////////
    
    ros::NodeHandle nh("~");  // Create a private node handle to access parameters

    // Get parameters from the parameter server
    int simulationFrequency;
    int visualizationFrequency;
    std::string xmlLocation;
    std::string controlMode;
    std::string publisherName;
    std::string endEffectorState_publisherName;
    std::string subscriberName;
    std::string endEffectorName;
    std::string pluginDirectory;
    double proportionalGain;
    double derivativeGain;
    double integralGain;
    std::vector<double> camera_focal_point;
    double camera_distance;
    double camera_azimuth;
    double camera_elevation;
    bool camera_orthographic;

    // Retrieve the parameters, providing default values if they are not set
    nh.param<int>("simulation_frequency", simulationFrequency, 1);
    nh.param<int>("visualization_frequency", visualizationFrequency, 20);
    nh.param<std::string>("xml_path", xmlLocation, "");
    nh.param<std::string>("control_mode", controlMode, "TORQUE");
    nh.param<std::string>("publisher_name", publisherName, "joint_states");
    nh.param<std::string>("eef_state_publisher_name", endEffectorState_publisherName, "eef_pose");
    nh.param<std::string>("subscriber_name", subscriberName, "joint_commands");
    nh.param<std::string>("end_effector_name", endEffectorName, "tool_link_ee");
    nh.param<std::string>("plugin_directory", pluginDirectory, "../../mujoco/mujoco-3.2.3/bin");

    nh.param<double>("proportional_gain", proportionalGain, 1.0);
    nh.param<double>("derivative_gain", derivativeGain, 0.01);
    nh.param<double>("integral_gain", integralGain, 0.0);

    nh.param<std::vector<double>>("camera_focal_point", camera_focal_point, std::vector<double>({0.0, 0.0, 0.5}));
    nh.param<double>("camera_distance", camera_distance, 2.5);
    nh.param<double>("camera_azimuth", camera_azimuth, 135);
    nh.param<double>("camera_elevation", camera_elevation, -30);
    nh.param<bool>("camera_orthographic", camera_orthographic, false);

    // Set the control mode
    ControlMode control_mode;
         if (controlMode == "POSITION") control_mode = POSITION;
    else if (controlMode == "VELOCITY") control_mode = VELOCITY;
    else if (controlMode == "TORQUE")   control_mode = TORQUE;
    else                                control_mode = UNKNOWN;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    
    try
    {
        // Create the MuJoCo simulation interface
        auto mujocoSim = std::make_shared<MujocoInterface>(xmlLocation,
                                                           publisherName,
                                                           endEffectorState_publisherName,
                                                           subscriberName,
                                                           endEffectorName,
                                                           pluginDirectory,
                                                           control_mode,
                                                           simulationFrequency,
                                                           visualizationFrequency);

        // Set feedback gains
        mujocoSim->set_feedback_gains(proportionalGain, integralGain, derivativeGain);

        // Set camera properties
        mujocoSim->set_camera_properties({camera_focal_point[0], camera_focal_point[1], camera_focal_point[2]},
                                          camera_distance,
                                          camera_azimuth, 
                                          camera_elevation,
                                          camera_orthographic);
        
        // Run the simulation indefinitely
        ros::spin();
    }
    catch(const std::exception &exception)
    {
        std::cerr << exception.what() << "\n";
    }
    
    return 0;
}
