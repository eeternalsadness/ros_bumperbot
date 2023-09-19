# indicate the Python interpreter that's used for the script
# !/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    # initialize node with a name and set the name to be unique
    rospy.init_node('simple_publisher_py', anonymous=True)

    # create a publisher with a topic name, the type of data transmitted, and the buffer size
    pub = rospy.Publisher('chatter', String, queue_size=10)

    # set the frequency in hertz
    r = rospy.Rate(10)

    counter = 0
    # while the rospy is still running
    while not rospy.is_shutdown():
        hello_msg = "Hello World from Python: %d" % counter
        pub.publish(hello_msg)
        r.sleep()
        counter += 1