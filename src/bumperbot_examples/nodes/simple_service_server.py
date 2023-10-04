# !/usr/bin/env python3
import rospy
from bumperbot_examples.srv import AddTwoInts, AddTwoIntsResponse

def add_two_ints(req):
    rospy.loginfo("Ready to sum %d and %d", req.a, req.b)
    return AddTwoIntsResponse(req.a + req.b)

if __name__ == '__main__':
    rospy.init_node("simple_service_server_py")
    service = rospy.Service("add_two_ints", AddTwoInts, add_two_ints)
    rospy.loginfo("The service is ready to add ints")
    rospy.spin()