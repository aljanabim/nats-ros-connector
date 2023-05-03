import rospy
import asyncio
from io import BytesIO


class NATSPublisher:
    def __init__(self, nats_connection, topic_name, event_loop):
        self.nc = nats_connection
        self.topic_name = topic_name[1:] if topic_name.startswith("/") else topic_name
        # in NATS "." is used to separate tokens whereas in we namespace with "/"
        self.topic_name_nats = self.topic_name.replace("/", ".")

        self.event_loop = event_loop
        # Use AnyMsg to get a serialized message to be forwarded to another client
        self.sub = rospy.Subscriber(self.topic_name, rospy.AnyMsg, self.ros_cb)

    def ros_cb(self, msg):
        asyncio.run_coroutine_threadsafe(self.handle_msg(msg), self.event_loop).result()
        # calling .result() ensure waiting until the handle_msg completes
        # See https://answers.ros.org/question/362598/asyncawait-in-subscriber-callback/
        # and https://docs.python.org/3/library/asyncio-task.html#scheduling-from-other-threads

    async def handle_msg(self, msg):
        buff = BytesIO()
        msg.serialize(buff)
        await self.nc.publish(self.topic_name_nats, buff.getvalue())
