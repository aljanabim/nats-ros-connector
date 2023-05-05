#! /usr/bin/env python3
import rospy
from std_msgs.msg import String, Header
from std_srvs.srv import Trigger, SetBool


def handle_bool(req):
    return {"success": False, "message": f"Hello SetBool from Client"}


if __name__ == "__main__":
    ## Initialize node
    rospy.init_node("example_clientB")

    r = rospy.Rate(1 / 2)
    pub1 = rospy.Publisher("clientA_listener", String, queue_size=10)

    rospy.Subscriber(
        "clientA_talker",
        Header,
        callback=lambda msg: print("\nfrom clientA_talker\n", msg),
    )

    rospy.Service("trigger_clientB", SetBool, handle_bool)
    # sleep for 2 seconds to wait for NATS client advertising service to launch
    rospy.sleep(2)

    while not rospy.is_shutdown():
        # Call Service
        rospy.wait_for_service("trigger_clientA")
        service = rospy.ServiceProxy("trigger_clientA", Trigger)
        print(
            "\nSERVICE RESULT OF trigger_clientA\n",
            service(),
        )

        # Publish message
        msg1 = String()  # Header()
        msg1.data = str(rospy.Time.now())
        pub1.publish(msg1)
        r.sleep()
