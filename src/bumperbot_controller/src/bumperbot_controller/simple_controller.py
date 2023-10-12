# !/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import JointState
import math
from nav_msgs.msg import Odometry
import tf_conversions

class SimpleController(object):
    def __init__(self, wheel_radius, wheel_separation):
        rospy.loginfo("Using wheel radius %d" % wheel_radius)
        rospy.loginfo("Using wheel separation %d" % wheel_separation)
        self.wheel_radius_ = wheel_radius
        self.wheel_separation_ = wheel_separation
        self.left_wheel_prev_pos_ = 0.0
        self.rigth_wheel_prev_pos_ = 0.0
        self.prev_time_ = rospy.Time.now()
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # odometry message
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.right_cmd_pub_ = rospy.Publisher("wheel_right_controller/command", Float64, queue_size = 10)
        self.left_cmd_pub_ = rospy.Publisher("wheel_left_controller/command", Float64, queue_size = 10)
        self.odom_pub_ = rospy.Publisher("bumperbot_controller/odom", Odometry, queue_size=10)

        self.vel_sub_ = rospy.Subscriber("bumperbot_controller/cmd_vel", Twist, self.velCallback)
        self.joint_sub_ = rospy.Subscriber("joint_states", JointState, self.jointCallback)

        self.speed_conversion_ = np.array([[wheel_radius / 2, wheel_radius / 2], 
                                           [wheel_radius / wheel_separation, -wheel_radius / wheel_separation]])
        
        rospy.loginfo("The conversion matrix is %s" % self.speed_conversion_)

    def velCallback(self, msg):
        robot_speed = np.array([[msg.linear.x], 
                                [msg.angular.z]])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) 
        right_speed = Float64(wheel_speed[0, 0])
        left_speed = Float64(wheel_speed[1, 0])
        
        self.right_cmd_pub_.publish(right_speed)
        self.left_cmd_pub_.publish(left_speed)

    def jointCallback(self, msg):
        dp_left = msg.position[0] - self.left_wheel_prev_pos_
        dp_right = msg.position[1] - self.rigth_wheel_prev_pos_
        dt = (msg.header.stamp - self.prev_time_).to_sec()

        self.left_wheel_prev_pos_ = msg.position[0]
        self.rigth_wheel_prev_pos_ = msg.position[1]
        self.prev_time_ = msg.header.stamp

        phi_left = dp_left / dt
        phi_right = dp_right / dt

        linear = (self.wheel_radius_ * phi_right + self.wheel_radius_ * phi_left) / 2
        angular = (self.wheel_radius_ * phi_right - self.wheel_radius_ * phi_left) / self.wheel_separation_

        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = rospy.Time.now()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        self.odom_pub_.publish(self.odom_msg_)