import rospy
import asyncio
from io import BytesIO
import rosservice
import nats
from importlib import import_module

class NATSServiceProxy:
    def __init__(self, nats_connection, service_name, service_type, event_loop, req_timeout):
        self.nc = nats_connection
        self.service_name = (
            service_name[1:] if service_name.startswith("/") else service_name
        )
        self.service_name_with_slash = (
            service_name if service_name.startswith("/") else "/" + service_name
        )
        # in NATS "." is used to separate tokens whereas in we namespace with "/"
        self.service_name_nats = self.service_name.replace("/", ".")

        self.event_loop = event_loop

        # Get the service type and make a dynamic import
        self.service_type = service_type
        self.service_class = self.import_service_class()
        self.req_timeout = req_timeout

        # Create a service to meet the proxy on the ROS side.
        self.service = rospy.Service(
            self.service_name_with_slash, self.service_class, self.ros_cb
        )
        self.service_class = rosservice.get_service_class_by_name(
            self.service_name_with_slash
        )
        self.service_resp_class = self.service_class._response_class()
        # Use AnyMsg to get a serialized message to be forwarded to another client
        # self.sub = rospy.Subscriber(self.service_name, rospy.AnyMsg, self.ros_cb)

    def import_service_class(self):
        # Dynamic imports, see https://blog.gitnux.com/code/python-importlib/
        service_pkg, service_class_name = self.service_type.split("/")
        service_pkg += ".srv"
        service_module = import_module(service_pkg)
        return getattr(service_module, service_class_name)

    def ros_cb(self, req):
        return asyncio.run_coroutine_threadsafe(
            self.handle_req(req), self.event_loop
        ).result()
        # calling .result() ensure waiting until the handle_msg completes
        # See https://answers.ros.org/question/362598/asyncawait-in-subscriber-callback/
        # and https://docs.python.org/3/library/asyncio-task.html#scheduling-from-other-threads

    async def handle_req(self, req):
        buff = BytesIO()
        req.serialize(buff)
        try:
            # Request Service over NATS
            response = await self.nc.request(
                self.service_name_nats, buff.getvalue(), timeout=self.req_timeout
            )
            # Deserialize response into the service response class
            self.service_resp_class.deserialize(response.data)
            return self.service_resp_class
        except TimeoutError:
            print(
                f"NATS Service Proxy ERROR: REQUEST FOR {self.service_name_with_slash} TIMED OUT"
            )
        except nats.errors.NoRespondersError:
            print(
                f"NATS Service Proxy ERROR: REQUEST FOR [{self.service_name_with_slash}] has no NATS responders available (No NATS Client is advertising this service yet)"
            )
        # Return default response
        return rosservice.get_service_class_by_name(
            self.service_name_with_slash
        )._response_class()
