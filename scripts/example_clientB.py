#! /usr/bin/env python3
import rospy
from std_msgs.msg import String, Header

if __name__ == "__main__":
    ## Initialize node
    rospy.init_node("example_clientB")

    r = rospy.Rate(1)
    pub1 = rospy.Publisher("clientA_listener", String, queue_size=10)

    rospy.Subscriber(
        "clientA_talker",
        Header,
        callback=lambda msg: print("\nfrom clientA_talker\n", msg),
    )

    while not rospy.is_shutdown():
        msg1 = String()  # Header()
        msg1.data = str(rospy.Time.now())
        pub1.publish(msg1)
        r.sleep()
