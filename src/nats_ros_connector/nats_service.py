import rospy
import rosservice
from io import BytesIO


class NATSService:
    def __init__(self, nats_connection, service_name):
        self.nc = nats_connection
        self.service_name = (
            service_name[1:] if service_name.startswith("/") else service_name
        )
        self.service_name_with_slash = (
            service_name if service_name.startswith("/") else "/" + service_name
        )
        # in NATS "." is used to separate tokens whereas in we namespace with "/"
        self.service_name_nats = self.service_name.replace("/", ".")

        self.service_class = None
        self.service_req_class = None
        self.service_resp_class = None

    def parse_msg(self, msg):
        _ = msg.subject  # placeholder for NATS subject
        _ = msg.reply  # placeholder for NATS reply
        data = msg.data  # no need for .decode()
        return data

    async def nats_msg_cb(self, msg):
        data = self.parse_msg(msg)

        rospy.wait_for_service(self.service_name_with_slash)
        if self.service_class is None:
            self.service_class = rosservice.get_service_class_by_name(
                self.service_name_with_slash
            )
            self.service_req_class = self.service_class._request_class()

        if self.service_class is not None:
            # Create a new Service Proxy for every incoming request
            service = rospy.ServiceProxy(
                self.service_name_with_slash, self.service_class, persistent=False
            )
            # Deserialize request and call ROS service
            try:
                self.service_req_class.deserialize(data)
                # Get the service response
                resp = service(self.service_req_class)
                # Serialize response and send back to client
                buff = BytesIO()
                resp.serialize(buff)
                await msg.respond(buff.getvalue())
            except Exception as e:
                print(
                    f"NATS Error when deserializing an incoming service request for [{self.service_name}]: {e}"
                )

    async def run(self):
        await self.nc.subscribe(self.service_name_nats, cb=self.nats_msg_cb)
