# !/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

class TurtlesimKinematics(object):
    def __init__(self):
        self.turtle1_pose_sub_ = rospy.Subscriber("/turtle1/pose", Pose, self.turtle1PoseCallback)
        self.turtle2_pose_sub_ = rospy.Subscriber("/turtle2/pose", Pose, self.turtle2PoseCallback)

        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

    def turtle1PoseCallback(self, pose):
        self.last_turtle1_pose_ = pose

    def turtle2PoseCallback(self, pose):
        self.last_turtle2_pose_ = pose
        
        Tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        Ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y

        rospy.loginfo("""---------------------------\n
                      Translation Vector turtle1 -> turtle2\n
                      Tx: %f\n
                      Ty: %f\n""",
                      Tx, Ty)