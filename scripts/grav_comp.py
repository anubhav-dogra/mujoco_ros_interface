import rospy
import tf.transformations
import tf2_geometry_msgs
import tf2_ros
import numpy as np
from geometry_msgs.msg import WrenchStamped
from scipy.signal import butter, filtfilt

class GravityCompensationNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('grav_comp', anonymous=True)
        self.rate =rospy.Rate(200)

        #butterworth filter parameters
        self.cutoff_frequency = 0.5  # Hz
        self.sampling_rate = 100.0  # Hz
        self.order = 2
        #filter coefficients
        self.b, self.a =self.butterworth_filter(self.cutoff_frequency, self.sampling_rate, self.order)

        self.window_size = 100

        # Initialize TF2 buffer and listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Publisher for compensated wrench
        self.pub = rospy.Publisher('/cartesian_wrench_tool_biased', WrenchStamped, queue_size=1)
        self.pub_check = rospy.Publisher('/cartesian_wrench_tool_unfiltered', WrenchStamped, queue_size=1)

        # Subscriber for force-torque sensor data
        rospy.Subscriber('/netft_data', WrenchStamped, self.callback_)

        # Precompute constants
        self.Fmg = np.array([0, 0, -3.1 * 9.81])  # Adjust force due to gravity
        self.zero_vec = np.zeros(3)
        self.P_s_g = np.array([
            [0, -0.005, 0.005],  # for some reason, com_Z is 10 times lesser then mujoco's com in xml 
            [0.005, 0.0, -0.015],
            [-0.005, 0.015, 0.0]
        ])
        # Form wrench vector
        self.wrench_vector = np.hstack((self.Fmg, self.zero_vec)).reshape(6, 1)

        self.force_data = []
        self.torque_data = []
        self.out = WrenchStamped()
        self.out_check = WrenchStamped()
        self.transformed_wrench = WrenchStamped()

    def butterworth_filter(self, cutoff_frequency, sampling_rate, order):
        nyquist_frequency = 0.5 * sampling_rate
        normalized_cutoff_frequency = cutoff_frequency / nyquist_frequency
        b, a = butter(order, normalized_cutoff_frequency, btype='low', analog=False)
        return b, a
    
    def apply_filter(self, data):
        if len(data) < self.order +1: #Ensure we have enough data
            return data
        #Apply butterworth filter
        return filtfilt(self.b, self.a, data)
    
    def get_gravity_wrench(self):
        try:
            # Lookup the transform from the base link to the sensor link
            transformStamped = self.tfBuffer.lookup_transform('iiwa_link_0', 'sensor_link', rospy.Time(0))

            # Extract translation (position)
            translation = transformStamped.transform.translation
            x, y, z = translation.x, translation.y, translation.z

            # Extract orientation (quaternion)
            rotation = transformStamped.transform.rotation
            qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w

            # Convert quaternion to rotation matrix
            q = [qx, qy, qz, qw]
            R = tf.transformations.quaternion_matrix(q)[:3, :3].T # with respect to sensor frame I guess, thats why transposed
            # R = R_ @ np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
            # Construct the spatial transformation matrix F_s_g
            F_s_g = np.block([
                [R, 0*np.eye(3)],           # Upper half (rotation and identity)
                [(self.P_s_g @ R), R]  # Lower half (cross product and rotation)
            ])

            # Compute the compensated wrench
            result = F_s_g @ self.wrench_vector # it was negative first! (coz sensor reading is opposite of real sensor)

            # print(result) 
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error in lookup transform: {e}")
        # print(result)
        return result

    def get_wrench_tool(self, wrench_in):
       try:
        s_eef_transformStamped = self.tfBuffer.lookup_transform('tool_link_ee','sensor_link', rospy.Time(0))
        wrench_out = WrenchStamped()
        wrench_out = tf2_geometry_msgs.do_transform_wrench(wrench_in, s_eef_transformStamped)
        return wrench_out
       except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error in lookup transform: {e}")

    def callback_(self, msg): 

        # Append new force and torque data to buffers
        # sensor values are opposite of real sensors, thats why negative sign is added.  
        self.force_data.append([-msg.wrench.force.x, -msg.wrench.force.y, -msg.wrench.force.z])
        self.torque_data.append([-msg.wrench.torque.x, -msg.wrench.torque.y, -msg.wrench.torque.z])

        if len(self.force_data) > self.sampling_rate:
            self.force_data.pop(0)
            self.torque_data.pop(0)
        else:
            return

        # if len(self.force_data) >= self.order + 1:
       
        # Get the latest filtered data
        # filtered_force = self.apply_filter(np.array(self.force_data).T).T[-1]
        # filtered_torque = self.apply_filter(np.array(self.torque_data).T).T[-1]
        filtered_force = np.mean(self.force_data[-self.window_size:], axis=0)  # Moving average
        filtered_torque = np.mean(self.torque_data[-self.window_size:], axis=0)

        result = self.get_gravity_wrench()

        self.out.header = msg.header
        self.out.wrench.force.x     = filtered_force[0]   - result[0]
        self.out.wrench.force.y     = filtered_force[1]   - result[1]
        self.out.wrench.force.z     = filtered_force[2]   - result[2]
        self.out.wrench.torque.x    = filtered_torque[0]  - result[3]
        self.out.wrench.torque.y    = filtered_torque[1]  - result[4]
        self.out.wrench.torque.z    = filtered_torque[2]  - result[5]

        self.transformed_wrench = self.get_wrench_tool(self.out)
        self.transformed_wrench.header.stamp = rospy.Time.now()

        self.out_check.header = msg.header
        self.out_check.wrench.force.x     = -msg.wrench.force.x   - result[0]
        self.out_check.wrench.force.y     = -msg.wrench.force.y   - result[1]
        self.out_check.wrench.force.z     = -msg.wrench.force.z   - result[2]
        self.out_check.wrench.torque.x    = -msg.wrench.torque.x  - result[3]
        self.out_check.wrench.torque.y    = -msg.wrench.torque.y  - result[4]
        self.out_check.wrench.torque.z    = -msg.wrench.torque.z  - result[5]


    def publish(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.out)
            self.pub_check.publish(self.out_check)
            self.rate.sleep()

if __name__ == '__main__':
    node = GravityCompensationNode()
    node.publish()
