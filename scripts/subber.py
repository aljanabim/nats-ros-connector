#! /usr/bin/env python3
import rospy
from std_msgs.msg import Header

if __name__ == "__main__":
    ## Initialize node
    rospy.init_node("subber")
    rospy.Subscriber("listener1", Header, callback=lambda msg: print(msg))
    rospy.Subscriber("listener2", Header, callback=lambda msg: print(msg))
    rospy.spin()
