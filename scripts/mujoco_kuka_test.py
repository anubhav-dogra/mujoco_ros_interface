import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def publisher():
    rospy.init_node('kuka_joint_command_publisher', anonymous=True)
    joint_command_pub = rospy.Publisher('/mujoco_interface_node/joint_commands', Float64MultiArray,queue_size=1)
    rate = rospy.Rate(10)  # 10 Hz
    joint_commands = Float64MultiArray()
    joint_commands.data = [0.0] * 7  # 7-DOF robot (initialize with zeroes)


    while not rospy.is_shutdown():
        joint_commands.data[0] = 0  # Joint 1 command
        joint_commands.data[1] = 0.5 # Joint 2 command
        joint_commands.data[2] = 0.0 # Joint 3 command
        joint_commands.data[3] = 0.0  # Joint 4 command
        joint_commands.data[4] = 0.0  # Joint 5 command
        joint_commands.data[5] = 0.0 # Joint 6 command
        joint_commands.data[6] = 0.0  # Joint 7 command
        joint_command_pub.publish(joint_commands)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass