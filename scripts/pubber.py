#! /usr/bin/env python3
import rospy
from std_msgs.msg import Header

if __name__ == "__main__":
    ## Initialize node
    rospy.init_node("pubber")
    r = rospy.Rate(10)
    pub1 = rospy.Publisher("talker1", Header, queue_size=10)
    pub2 = rospy.Publisher("talker2", Header, queue_size=10)

    rospy.Subscriber("listener2", Header, callback=lambda msg: print(msg))

    while rospy.is_shutdown() != True:
        msg1 = Header()
        msg1.stamp = rospy.Time.now()
        print(msg1)
        pub1.publish(msg1)

        msg2 = Header()
        msg2.stamp = rospy.Time.now()
        pub2.publish(msg2)

        # pub1.publish(f"talker 1: {rospy.Time.now()}")
        # pub2.publish(f"talker 2: {rospy.Time.now()}")
        r.sleep()
