import rospy
import asyncio
from io import BytesIO
from std_msgs.msg import Header


class NATSPublisher:
    def __init__(self, nats_connection, topic_name, rate=30):
        self.nc = nats_connection
        self.topic_name = topic_name
        self.rate = rate

        self.msg = None
        # Use AnyMsg to get a serialized message to be forwarded to another client
        self.sub = rospy.Subscriber(self.topic_name, rospy.AnyMsg, self.ros_cb)
        # TODO Make the ros_cb trigger a create_task instead of running an infinite loop
        # To make sure that the rate is not needed.
        # See https://answers.ros.org/question/362598/asyncawait-in-subscriber-callback/

    def ros_cb(self, msg):
        self.msg = msg

    async def run(self):
        while not rospy.is_shutdown():
            if self.msg is not None:
                buff = BytesIO()
                self.msg.serialize(buff)
                await self.nc.publish(self.topic_name, buff.getvalue())
                self.msg = None
            else:  # sleep to keep event loop running
                await asyncio.sleep(1 / self.rate)
