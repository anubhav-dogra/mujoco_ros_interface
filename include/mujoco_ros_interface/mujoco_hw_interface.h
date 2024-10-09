#ifndef MUJOCO_HW_INTERFACE_H
#define MUJOCO_HW_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include "MujocoInterface.h"

namespace mujoco_hardware_interface {
    
class MujocoHwInterface : public hardware_interface::RobotHW {
public:
    // MujocoHwInterface(MujocoInterface& mujocoInterface);

    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
    void jointCommandCallback(const std_msgs::Float64MultiArrayConstPtr& msg);
    bool read(const ros::Time time, const ros::Duration period);
    bool write(const ros::Time time, const ros::Duration period);


private:
    // MujocoInterface& mujocoInterface_;

    hardware_interface::JointStateInterface joint_state_interface_;
    // hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;


    ros::Subscriber joint_state_sub_;
    ros::Subscriber controller_sub_;
    ros::Publisher mujoco_command_pub_;
  
    std::vector<std::string> jointNames_;

    int numJoints_;

    int SimFrequency;
    std::vector<double> jointPositions_;
    std::vector<double> jointVelocities_;
    std::vector<double> jointEfforts_;
    std::vector<double> jointCommands_;

     // Latest received joint state data
    std::vector<double> latest_position_;
    std::vector<double> latest_velocity_;
    std::vector<double> latest_effort_;
};
}

#endif // MUJOCO_HW_INTERFACE_H