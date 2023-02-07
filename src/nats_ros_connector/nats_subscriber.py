import rospy
import rostopic


class NATSSubscriber:
    def __init__(self, nats_connection, topic_name, topic_type):
        self.nc = nats_connection
        self.topic_name = topic_name
        self.topic_type = topic_type

        # Use AnyMsg to get a serialized message to be forwarded to another client
        self.pub = rospy.Publisher(self.topic_name, rospy.AnyMsg, queue_size=10)

    def parse_msg(self, msg):
        subject = msg.subject
        reply = msg.reply
        data = msg.data.decode()

        # print(f"Received a message on {subject} {reply}: {data}")
        return data

    async def run(self):
        # await self.nc.subscribe(self.topic_name, cb=self.cb) # a callback-based implementation causes the event loop to close prematurely
        sub = await self.nc.subscribe(self.topic_name)
        while not rospy.is_shutdown():
            try:
                msg = await sub.next_msg()
                data = self.parse_msg(msg)
                # Get the topic class if available
                # TODO support topics beginning with "/"
                Msg, _, _ = rostopic.get_topic_class(f"/{self.topic_name}")
                if Msg is not None:
                    # print(self.topic_name, Msg)
                    # TODO test serialization
                    m = Msg()
                    m.deserialize(data)
                    self.pub.publish(data)
            except:
                # Continue ensure that we re-subscribe in-case of no subscriber available
                continue
