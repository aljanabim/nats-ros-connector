import rospy
import rostopic


class NATSSubscriber:
    def __init__(self, nats_connection, topic_name):
        self.nc = nats_connection
        self.topic_name = topic_name[1:] if topic_name.startswith("/") else topic_name
        self.topic_name_with_slash = (
            topic_name if topic_name.startswith("/") else "/" + topic_name
        )
        # in NATS "." is used to separate tokens whereas in we namespace with "/"
        self.topic_name_nats = self.topic_name.replace("/", ".")
        self.pub = None

    def parse_msg(self, msg):
        _ = msg.subject  # placeholder for NATS subject
        _ = msg.reply  # placeholder for NATS reply
        data = msg.data  # no need for .decode()
        return data

    async def nats_msg_cb(self, msg):
        data = self.parse_msg(msg)

        # Get the topic class if available
        Msg, _, _ = rostopic.get_topic_class(self.topic_name_with_slash)

        if Msg is not None:
            # Instantiate a ROS Publisher with the topic class of a ROS Subscriber already registered
            if self.pub is None:
                self.pub = rospy.Publisher(self.topic_name, Msg, queue_size=1, tcp_nodelay=True)
            # Deserialize message and publish
            if self.pub is not None:
                try:
                    m = Msg()
                    m.deserialize(data)
                    self.pub.publish(m)
                except Exception as e:
                    print(f"NATS Error when decoding [{self.topic_name}]: {e}")

    async def run(self):
        await self.nc.subscribe(self.topic_name_nats, cb=self.nats_msg_cb)
