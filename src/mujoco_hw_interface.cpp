#include "mujoco_ros_interface/mujoco_hw_interface.h"

namespace mujoco_hardware_interface
{

    bool MujocoHwInterface::init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle& robot_hw_nh)
    {
    // if (!robot_hw_nh.getParam("joint_names", jointNames_))
    // {
    //     ROS_ERROR("Cannot find required parameter 'joint_names' on the parameter server.");
    //     throw std::runtime_error("Cannot find required parameter 'joint_names' on the parameter server.");
    // }
    jointNames_ = {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};

    numJoints_ = jointNames_.size();
    // ROS_INFO_NAMED("MujocoHwInterface", "Found %zu joints", numJoints_);

    jointPositions_.resize(numJoints_, std::numeric_limits<double>::quiet_NaN());
    jointVelocities_.resize(numJoints_, std::numeric_limits<double>::quiet_NaN());
    jointEfforts_.resize(numJoints_, std::numeric_limits<double>::quiet_NaN());
    jointCommands_.resize(numJoints_, std::numeric_limits<double>::quiet_NaN());
    latest_position_.resize(numJoints_, std::numeric_limits<double>::quiet_NaN());
    latest_velocity_.resize(numJoints_, std::numeric_limits<double>::quiet_NaN());
    latest_effort_.resize(numJoints_, std::numeric_limits<double>::quiet_NaN());

    for (size_t i = 0; i < numJoints_; i++)
        {
            joint_state_interface_.registerHandle(
                hardware_interface::JointStateHandle(
                    jointNames_[i], &jointPositions_[i], &jointVelocities_[i], &jointEfforts_[i]));

            effort_joint_interface_.registerHandle(hardware_interface::JointHandle(
                joint_state_interface_.getHandle(jointNames_[i]), &jointCommands_[i]));
        }
        registerInterface(&joint_state_interface_);
        registerInterface(&effort_joint_interface_);

        ROS_INFO_NAMED("MujocoHwInterface", "Starting ...");
        
        // controller_sub_ = robot_hw_nh.subscribe("/random_command", 10, &MujocoHwInterface::jointCommandCallback, this);

        joint_state_sub_ = robot_hw_nh.subscribe("/joint_states", 10, &MujocoHwInterface::jointStateCallback, this);
        
        mujoco_command_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("/mujoco_interface_node/joint_commands", 10);
        return true;

        
    }

    void MujocoHwInterface::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
    {
        latest_position_ = msg->position;
        latest_velocity_ = msg->velocity;
        latest_effort_  = msg->effort;
    }

    // void MujocoHwInterface::jointCommandCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
    // {
    //     jointCommands_ = msg->data;
    // }

    bool MujocoHwInterface::read(const ros::Time time, const ros::Duration period)
    {
        for (size_t i = 0; i < numJoints_; i++)
        {
            jointPositions_[i] = latest_position_[i];
            jointVelocities_[i] = latest_velocity_[i];
            jointEfforts_[i] = latest_effort_[i];
        }

        return true;
    }

    bool MujocoHwInterface::write(const ros::Time time, const ros::Duration period)
    {

        std_msgs::Float64MultiArray msg;
        msg.data = jointCommands_;  // jointCommands_ are set in via effort_joint_interface by the controllers !!
        mujoco_command_pub_.publish(msg);
        
        return true;
    }
        
}