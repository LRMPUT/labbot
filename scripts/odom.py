from math import atan2
import csv
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import numpy as np

# TARGET MANUALLY SET
target_pos = [0.0, 0.0, 0.0, 0.0]  # x, y, z, yaw
current_pos = [0.0, 0.0, 0.0, 0.0]  # x, y, z, yaw
angle_difference = 0.0

def odom_callback(data):
    global current_pos, target_pos, angle_difference
    current_pos[0] = data.pose.pose.position.x
    current_pos[1] = data.pose.pose.position.y
    # current_pos[2] = data.pose.pose.position.z
    (ar, ap, yaw_val) = tf.transformations.euler_from_quaternion(
		[data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
		 data.pose.pose.orientation.w])
    current_pos[3] = np.rad2deg(yaw_val)
    
    angle_from_labbot_center = atan2(target_pos[1] - current_pos[1], target_pos[0] - current_pos[0])
    angle_difference = angle_from_labbot_center - current_pos[3]

def read_pose(data):
    global current_pos, target_pos, angle_difference
    print("x position:  " + str(current_pos[0]) + ",    x-target-diff:    " + str(target_pos[0] - current_pos[0])) # x-diff, distance to x
    print("y position:  " + str(current_pos[1]) + ",    y-target-diff:    " + str(target_pos[1] - current_pos[1])) # y-diff, distance to y
    print("yaw angle:   " + str(current_pos[3]) + ",    yaw-target-diff:  " + str(angle_difference)) # angle diff
    print("----------------------------")


def labbot_controller():
    rospy.init_node('labbot_controller')
    odom_sub = rospy.Subscriber('/labbot_odometry', Odometry, odom_callback)
    pose = rospy.Subscriber('/pose_reading', Empty, read_pose)
    rospy.loginfo('Node activated!')
    rospy.spin()
    rospy.loginfo('Node closed!')


if __name__ == "__main__":
    labbot_controller()
