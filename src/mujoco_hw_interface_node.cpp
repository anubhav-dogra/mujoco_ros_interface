#include "controller_manager/controller_manager.h"
#include "mujoco_ros_interface/mujoco_hw_interface.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iiwa");

    // Create an asynchronous spinner to handle ROS callbacks in a separate thread
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Create ROS node handles for the root and robot namespaces
    ros::NodeHandle root_nh;
    ros::NodeHandle nh("~");
    
    // create instance of mujoco hardware interface
    mujoco_hardware_interface::MujocoHwInterface mujoco_hardware_interface_;

    // create contrller manager and associate it with the mujoco hardware interface
    controller_manager::ControllerManager controller_manager(&mujoco_hardware_interface_, nh);
    
    // Intialize variables for timing
    ros::Time timestamp;
    ros::Duration period;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;

    //initialize mujoco hardware interface
    mujoco_hardware_interface_.init(root_nh, nh);


    ros::Rate rate(500);

    while(ros::ok())
    {
        if(!mujoco_hardware_interface_.read(timestamp, period)){
            ROS_FATAL_NAMED("mujoco_hardware_interface", 
                            "mujoco_hardware_interface failed to read robot state");
            ros::shutdown();
        }
  

    // Get the current time and calculate the elapsed time since the last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(
        stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    // Update the controllers with the current time and period
    controller_manager.update(timestamp, period);

    mujoco_hardware_interface_.write(timestamp, period);


    rate.sleep();
    }
    spinner.stop();
    return 0;
}
