#! /usr/bin/env python3
import rospy
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger, SetBool


def handle_trigger(req):
    return {"success": True, "message": f"Hello Trigger from Client A"}


if __name__ == "__main__":
    ## Initialize node
    rospy.init_node("example_clientA")

    r = rospy.Rate(1)
    pub1 = rospy.Publisher("clientA_talker", Header, queue_size=10)

    rospy.Subscriber(
        "clientA_listener",
        String,
        callback=lambda msg: print("\nfrom clientA_listener\n", msg),
    )

    rospy.Service("trigger_clientA", Trigger, handle_trigger)
    # sleep for 2 seconds to wait for NATS client advertising service to launch
    rospy.sleep(2)

    while not rospy.is_shutdown():
        # Call Service
        rospy.wait_for_service("trigger_clientB")
        service = rospy.ServiceProxy("trigger_clientB", SetBool)
        print(
            "\nSERVICE RESULT OF trigger_clientB\n",
            service(),
        )

        # Publish message
        msg1 = Header()
        msg1.stamp = rospy.Time.now()
        pub1.publish(msg1)
        r.sleep()
