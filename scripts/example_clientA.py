#! /usr/bin/env python3
import rospy
from std_msgs.msg import Header, String

if __name__ == "__main__":
    ## Initialize node
    rospy.init_node("example_clientA")

    r = rospy.Rate(10)
    pub1 = rospy.Publisher("clientA_talker", Header, queue_size=10)

    rospy.Subscriber(
        "clientA_listener",
        String,
        callback=lambda msg: print("\nfrom clientA_listener\n", msg),
    )

    while not rospy.is_shutdown():
        msg1 = Header()
        msg1.stamp = rospy.Time.now()
        pub1.publish(msg1)
        r.sleep()
