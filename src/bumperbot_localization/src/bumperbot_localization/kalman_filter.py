# !/usr/env/bin python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(object):
    def __init__(self):
        self.odom_sub_ = rospy.Subscriber("bumperbot_controller/odom_noisy", Odometry, self.odomCallback)
        self.imu_sub_ = rospy.Subscriber("imu", Imu, self.imuCallback)
        self.odom_pub_ = rospy.Publisher("bumperbot_controller/odom_kalman", Odometry, queue_size=10)

        self.mean_ = 0.0
        self.variance_ = 1000.0
        self.imu_angular_z_ = 0.0
        self.is_first_odom_ = True
        self.last_angular_z = 0.0
        self.motion_ = 0.0
        self.kalman_odom_ = Odometry()

    def imuCallback(self, imu):
        self.imu_angular_z_ = imu.angular_velocity.z

    def odomCallback(self, odom):
        self.kalman_odom_ = odom

        if self.is_first_odom_:
            self.mean_ = odom.twist.twist.angular.z
            self.last_angular_z = odom.twist.twist.angular.z
            self.is_first_odom_ = False
            return
        
        self.statePrediction()
        self.measurementUpdate()