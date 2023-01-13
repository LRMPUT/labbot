
# https://www.youtube.com/watch?v=0k17sg33t7A&list=PL8YzxXJHBHvGgcizTsVYVGcH2PmijEUGj&index=19&ab_channel=AiRVsem
from math import atan2, sqrt
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
target_pos = [2.29, -0.06, 0.0, 0.0]  # x, y, z, yaw
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


def control(pub):
    global target_pos, current_pos
    vel_msg = Twist()
    err_x = target_pos[0] - current_pos[0]
    err_y = target_pos[1] - current_pos[1]
    steering_angle = atan2(err_y, err_x)
    distance = sqrt(err_x**2 + err_y**2)

    if abs(distance) < 0.1:
        print("goal reached!\n")
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        pub.publish(vel_msg)
        return

    k_linear = 0.4
    k_angular = 0.2

    lin_vel = k_linear * distance
    angular_vel = k_angular * (steering_angle - current_pos[3])
#    if abs(angular_vel) >= 0.2:
 #       angular_vel = 0.2

    lin_vel = lin_vel if lin_vel <= 0.25 else 0.25
    angular_vel = angular_vel if abs(angular_vel) >= 0.05 else 0.0
    if angular_vel >0 and angular_vel > 0.2:
	angular_vel = 0.2
    elif angular_vel < 0 and angular_vel < -0.2:
	angular_vel = -0.2

#    vel_msg.linear.x = lin_vel
#    vel_msg.angular.z = angular_vel

   # pub.publish(vel_msg)
    print("Error x: " + str(err_x))
    print("Error y: " + str(err_y))
    print("Current yaw angle: " + str(current_pos[3]))
    print("Angle error: " + str(steering_angle - current_pos[3]))
    print("Calculated distance: " + str(distance))
    print("Suggested velocities: linear: " + str(lin_vel) + ", angular: "+ str(angular_vel)+ "\n") 

if __name__ == '__main__':
    rospy.init_node('labbot_speeding')
    rate = rospy.Rate(1)
    odom_sub = rospy.Subscriber('/labbot_odometry', Odometry, odom_callback)
    publisher = rospy.Publisher('/Twist', Twist, queue_size=10)
    rospy.loginfo('Node activated!')

    while not rospy.is_shutdown():
        control(publisher)
        rate.sleep()

    rospy.loginfo('Node closed!')
