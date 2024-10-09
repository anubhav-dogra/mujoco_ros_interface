import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R

class CartesianImpedanceControllerMujoco:
    def __init__(self):
        rospy.init_node('CartesianImpedanceMujoco', anonymous=True)

        self.Kp = np.diag([800, 800, 400, 100, 100, 100])
        self.Kd = 2 * np.sqrt(self.Kp)

        self.current_pose = PoseStamped()
        self.desired_pose = PoseStamped()

        rospy.Subscriber('/mujoco_interface_node/eef_pose', Float64MultiArray, self.current_pose_callback)
        rospy.Subscriber('/cartesian_trajectory_generator/ref_pose', PoseStamped, self.desired_pose_callback)
        
        self.torque_command_publisher = rospy.Publisher('/mujoco_interface_node/joint_commands', Float64MultiArray, queue_size=10)


    def current_pose_callback(self, msg):
        self.current_pose = msg
        # print(self.current_pose)

    def desired_pose_callback(self, msg):
        self.desired_pose = msg
        # print(self.desired_pose)

    def calculate_orientation_error(O_d, O_c):
        """
        Calculate the orientation error in axis-angle representation between two quaternions.
        
        :param orientation_d: Desired orientation (quaternion as [x, y, z, w])
        :param orientation: Current orientation (quaternion as [x, y, z, w])
        :return: Orientation error as a 3D vector (axis * angle)
        """
        o_d = np.array(O_d)
        o_c = np.array(O_c)

        if np.dot(o_d, o_c) < 0:
            o_c = -o_c

        r_d = R.from_quat(o_d)
        r_c = R.from_quat(o_c)

        error_quat = r_c * r_d.inv()

        # Convert difference quaternion to axis-angle
        error_axis_angle = error_quat.as_rotvec() # Get rotation vector (axis * angle)

        return error_axis_angle

    def compute_pose_error(self):
        error = np.zeros(6)

        error[:3] = np.array([-self.desired_pose.pose.position.x + self.current_pose.pose.position.x,
                              -self.desired_pose.pose.position.y + self.current_pose.pose.position.y,
                              -self.desired_pose.pose.position.z + self.current_pose.pose.position.z])
        
        O_d =   [
                self.desired_pose.pose.orientation.x,
                self.desired_pose.pose.orientation.y,
                self.desired_pose.pose.orientation.z,
                self.desired_pose.pose.orientation.w
                ]
        
        O_c =   [
                self.current_pose.pose.orientation.x,
                self.current_pose.pose.orientation.y,
                self.current_pose.pose.orientation.z,
                self.current_pose.pose.orientation.w
                ]   
        error[3:] = self.calculate_orientation_error(O_d, O_c)


        return error
