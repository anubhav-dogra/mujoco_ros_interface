#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>


double desired_force = 3.0;

class ContactForceControl{
    public:
        ContactForceControl(ros::NodeHandle& nh)
        : tfBuffer(), tfListener(tfBuffer) // Initialize tfBuffer and tfListener as class members
        {
            // ros::param::get("end_effector_name", end_effector_name);
            // tf2_ros::Buffer tfBuffer;
            // tf2_ros::TransformListener tfListener(tfBuffer);

            try
            {
                transformStamped_base_to_end = tfBuffer.lookupTransform("iiwa_link_0", "tool_link_ee", ros::Time(0), ros::Duration(1.0));
            }
            catch(tf2::TransformException &e)
            {
                ROS_ERROR("%s", e.what());
            }
            std::cout << transformStamped_base_to_end << std::endl;           
            tf2::convert(transformStamped_base_to_end.transform.rotation, previous_quat);
            previous_quat.normalize();
            pub_ = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/CartesianImpedance_trajectory_controller/reference_pose", 1);
            sub_ = nh.subscribe("/cartesian_wrench_tool", 1, &ContactForceControl::callback, this);
            
        }

        void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
        {
            current_force_z  = msg->wrench.force.z;
            current_force_x = msg->wrench.force.x;
            current_force_y = msg->wrench.force.y;
            torque_x = msg->wrench.torque.x;
            torque_y = msg->wrench.torque.y;
            
            error_fz = desired_force - current_force_z;
            double roll_error =  current_force_x;
            double pitch_error = -current_force_y;
            // double roll_error =  torque_x;
            // double pitch_error = torque_y;
            if (first_run)
            {
                previous_error_fz = error_fz;
                previous_error_roll = roll_error;
                previous_error_pitch = pitch_error;
                prev_time = ros::Time::now();
                first_run = false;
            }

             // Calculate the time difference
            current_time = ros::Time::now();
            double dt = (current_time - prev_time).toSec();
            prev_time = current_time;
            if (dt>0)
            {
                // error_integral += error_fz*dt;
                error_derivative = (error_fz - previous_error_fz)/dt;

                error_d_roll = (roll_error - previous_error_roll)/dt;
                error_d_pitch = (pitch_error - previous_error_pitch)/dt;
                smoothed_dFe = alpha*smoothed_dFe + (1-alpha)*error_derivative;
                // smoothed_dFe = alpha*smoothed_dFe + (1-alpha)*error_derivative;
                previous_error_fz = error_fz;
                previous_error_roll = roll_error;
                previous_error_pitch = pitch_error;

                // double approach_factor = std::min(fabs(error_fz) / desired_force, 1.0); // Scale factor (0 to 1)
                // double Kp_adjusted = Kp * approach_factor*0.1; // Reduce Kp as error reduces
                // ROS_INFO_STREAM("approach factor: " << approach_factor << " Kp_adjusted: " << Kp_adjusted);
                dZ = Kp*error_fz + Kd*smoothed_dFe;
                double max_dZ = 0.0005;  // Maximum allowed movement per iteration
                if (fabs(dZ) > max_dZ) {
                    dZ = copysign(max_dZ, dZ);  // Clamp the displacement
                    // std::cout << "Clamped dZ" << dZ << std::endl;
                }
                // ROS_INFO("dz: %f", dZ);

                // double roll_error = torque_y; // Roll error from torque around Y
                // double pitch_error = torque_x; // Pitch error from torque around X

                // Calculate orientation adjustments
                // double dRoll = 1*Kp_orientation_x * pitch_error + Kd_orientation * error_d_pitch + Kp_orientation_x *(0-torque_x);
                // double dPitch = 1*Kp_orientation_y * roll_error + Kd_orientation * error_d_roll + Kp_orientation_y * (0-torque_y);
                // Clamp dRoll and dPitch to limit abrupt changes
                
                double dRoll =  Kp_orientation_x *roll_error;
                double dPitch = Kp_orientation_y * pitch_error;
                double max_dRoll = 0.001;  // Maximum allowed roll adjustment
                double max_dPitch = 0.001; // Maximum allowed pitch adjustment

                if (fabs(dRoll) > max_dRoll) {
                    dRoll = copysign(max_dRoll, dRoll);
                    std::cout << "Clamped dRoll" << dRoll << std::endl;
                }

                if (fabs(dPitch) > max_dPitch) {
                    dPitch = copysign(max_dPitch, dPitch);
                    std::cout << "Clamped dPitch" << dPitch << std::endl;
                }            
            
                geometry_msgs::PoseStamped pose = update_pose(dZ, dRoll, dPitch);
                // std::cout << pose << std::endl;
                pub_.publish(pose);
            } 
            running_rate.sleep();

        }  

        bool isAbruptChange(const tf2::Quaternion& previous, const tf2::Quaternion& current, double threshold = 0.95) {
            // Calculate the dot product
            double dot_product = previous.x() * current.x() +
                                previous.y() * current.y() +
                                previous.z() * current.z() +
                                previous.w() * current.w();

            // If the dot product is less than the threshold, it indicates a large change
            return std::abs(dot_product) < threshold;
            }    

        geometry_msgs::PoseStamped update_pose(double& dZ, double& dRoll, double& dPitch)
        {
            tf2::Quaternion quat_delta;
            quat_delta.setRPY(dRoll, dPitch, 0.0);
            quat_delta.normalize();

            new_quat = quat_delta*previous_quat;
            new_quat.normalize();
            // std::cout << "quat_delta: " << quat_delta.getX() << " " << quat_delta.getY() << " " << quat_delta.getZ() << " " << quat_delta.getW() << std::endl;

            // if (isAbruptChange(previous_quat, quat_delta)) {
            //     ROS_WARN("Large change in orientation detected");
            // }
            // else
            // { 
            transformStamped_goal.header.stamp = ros::Time::now();
            transformStamped_goal.header.frame_id = "tool_link_ee";
            transformStamped_goal.child_frame_id = "goal_point";
            // std::cout << "prev_z" << transformStamped_goal.transform.translation.z << std::endl;
            transformStamped_goal.transform.translation.x = 0.0;
            transformStamped_goal.transform.translation.y = 0.0;
            transformStamped_goal.transform.translation.z += dZ;
            // transformStamped_goal.transform.rotation = tf2::toMsg(new_quat);
            transformStamped_goal.transform.rotation.x = quat_delta.getX();
            transformStamped_goal.transform.rotation.y = quat_delta.getY();
            transformStamped_goal.transform.rotation.z = quat_delta.getZ();
            transformStamped_goal.transform.rotation.w = quat_delta.getW();
            // std::cout << transformStamped_goal << std::endl;
            tf2::Vector3 translation_goal(transformStamped_goal.transform.translation.x,
                                            transformStamped_goal.transform.translation.y,
                                            transformStamped_goal.transform.translation.z);

            tf2::Quaternion quat_tf_goal;
            tf2::convert(transformStamped_goal.transform.rotation, quat_tf_goal);
            quat_tf_goal.normalize();
            tf2::Transform transform_goal(quat_tf_goal,translation_goal);

            tf2::Vector3 translation_base_ee(transformStamped_base_to_end.transform.translation.x,transformStamped_base_to_end.transform.translation.y,transformStamped_base_to_end.transform.translation.z);
            tf2::Quaternion quat_tf_base_end;
            tf2::convert(transformStamped_base_to_end.transform.rotation, quat_tf_base_end);
            quat_tf_base_end.normalize();
            tf2::Transform transform_base_ee(quat_tf_base_end,translation_base_ee);

            tf2:: Transform transformed_goal_base = transform_base_ee*transform_goal;

            geometry_msgs::TransformStamped static_transform_goal_base;
            static_transform_goal_base.header.stamp = ros::Time::now();
            static_transform_goal_base.header.frame_id = "world";
            static_transform_goal_base.child_frame_id = "goal_frame";
            static_transform_goal_base.transform = tf2::toMsg(transformed_goal_base);

            
            geometry_msgs::PoseStamped pose_got;
            pose_got.header.frame_id="world";
            pose_got.header.stamp = ros::Time::now();
            pose_got.pose.position.x = static_transform_goal_base.transform.translation.x;
            pose_got.pose.position.y = static_transform_goal_base.transform.translation.y;
            pose_got.pose.position.z = static_transform_goal_base.transform.translation.z;
            pose_got.pose.orientation = tf2::toMsg(new_quat);
            // pose_got.pose.orientation.x = static_transform_goal_base.transform.rotation.x;
            // pose_got.pose.orientation.y = static_transform_goal_base.transform.rotation.y;
            // pose_got.pose.orientation.z = static_transform_goal_base.transform.rotation.z;
            // pose_got.pose.orientation.w = static_transform_goal_base.transform.rotation.w;
            // std::cout << pose_got << std::endl;
            // std::cout << new_quat.x() << " " << new_quat.y() << " " << new_quat.z() << " " << new_quat.w() << std::endl;
            previous_quat = new_quat;
            return pose_got;
            // }
            
        } 

    private:
        std::string end_effector_name;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        double current_force_z=0.0, error_fz=0.0, error_integral=0.0, error_derivative=0.0, previous_error_fz=0.0, dZ=0.0;
        double torque_x=0.0, torque_y=0.0, current_force_x=0.0, current_force_y=0.0, previous_error_roll=0.0, previous_error_pitch=0.0, error_d_roll=0.0, error_d_pitch=0.0;
        double alpha = 0.9;
        double smoothed_dFe = 0.0;
        bool first_run = true;
        ros::Rate running_rate = 100;
        ros::Time current_time, prev_time;
        double factor = 1000.0;
        double Kp = 1/factor;
        double Kd = 0.1/factor;
        double Ki = 0.1/factor;

        double Kp_orientation_x = 1/factor;
        double Kp_orientation_y = 1/factor;
        double Kd_orientation = 0.01/factor;
        tf2::Quaternion previous_quat;
        tf2::Quaternion new_quat;


        geometry_msgs::TransformStamped transformStamped_base_to_end;
        geometry_msgs::TransformStamped transformStamped_goal;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "contact_force_control");
    ros::NodeHandle nh;
    ContactForceControl fc(nh);
    ros::spin();
    return 0;
}