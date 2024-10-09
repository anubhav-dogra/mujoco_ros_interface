#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from matplotlib import pyplot as plt

pos_1 = []
time = []
pos_2 = []

rospy.init_node('kuka_joint_command_publisher', anonymous=True)

joint_command_pub = rospy.Publisher('/mujoco_interface_node/joint_commands', Float64MultiArray,queue_size=10)
rospy.sleep(0.5)
joint_commands = Float64MultiArray()
joint_commands.data = [0.0] * 2  # 7-DOF robot (initialize with zeroes)
joint_commands.data[0] = 1.047  # Joint 1 command
joint_commands.data[1] = 1.57 # Jo

joint_command_pub.publish(joint_commands)


def callback(data):
    # APPEND DATA FOR PLOTTING for each joint
    curr_time = rospy.Time.now()
    curr_time_in_secs = curr_time.to_sec()

    time.append(curr_time_in_secs)
    pos_1.append(data.position[0])
    pos_2.append(data.position[1])




rospy.Subscriber('mujoco_interface_node/joint_states', JointState, callback)



while not rospy.is_shutdown():
    rate = rospy.Rate(100)  # 10 Hz
    rate.sleep()

else:
    plt.plot(time,pos_1, 'b-')
    plt.plot(time,pos_2, 'r-')

    plt.show()



