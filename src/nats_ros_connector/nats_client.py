import asyncio
import nats
import rospy
from nats_ros_connector.nats_publisher import NATSPublisher
from nats_ros_connector.nats_subscriber import NATSSubscriber


class NATSClient:
    def __init__(
        self,
        nats_host,
        publishers,
        subscribers,
        event_loop,
        # NATS Connection parameters
        name=None,
        pedantic=False,
        verbose=False,
        allow_reconnect=True,
        connect_timeout=2,
        reconnect_time_wait=2,
        max_reconnect_attempts=60,
        ping_interval=120,
        max_outstanding_pings=2,
        dont_randomize=False,
        flusher_queue_size=1024,
        no_echo=False,
        tls=None,
        tls_hostname=None,
        user=None,
        password=None,
        token=None,
        drain_timeout=30,
        signature_cb=None,
        user_jwt_cb=None,
        user_credentials=None,
        nkeys_seed=None,
    ):
        self.host = nats_host
        self.publishers = publishers
        self.subscribers = subscribers
        self.tasks = []
        self.event_loop = event_loop

        # NATS Connection parameters
        self.name = name
        self.pedantic = pedantic
        self.verbose = verbose
        self.allow_reconnect = allow_reconnect
        self.connect_timeout = connect_timeout
        self.reconnect_time_wait = reconnect_time_wait
        self.max_reconnect_attempts = max_reconnect_attempts
        self.ping_interval = ping_interval
        self.max_outstanding_pings = max_outstanding_pings
        self.dont_randomize = dont_randomize
        self.flusher_queue_size = flusher_queue_size
        self.no_echo = no_echo
        self.tls = tls
        self.tls_hostname = tls_hostname
        self.user = user
        self.password = password
        self.token = token
        self.drain_timeout = drain_timeout
        self.signature_cb = signature_cb
        self.user_jwt_cb = user_jwt_cb
        self.user_credentials = user_credentials
        self.nkeys_seed = nkeys_seed

    async def run(self):
        self.nc = await nats.connect(
            # See https://nats-io.github.io/nats.py/modules.html#asyncio-client
            self.host,
            error_cb=self._error_cb,
            reconnected_cb=self._reconnected_cb,
            disconnected_cb=self._disconnected_cb,
            closed_cb=self._closed_cb,
            name=self.name,
            pedantic=self.pedantic,
            verbose=self.verbose,
            allow_reconnect=self.allow_reconnect,
            connect_timeout=self.connect_timeout,
            reconnect_time_wait=self.reconnect_time_wait,
            max_reconnect_attempts=self.max_reconnect_attempts,
            ping_interval=self.ping_interval,
            max_outstanding_pings=self.max_outstanding_pings,
            dont_randomize=self.dont_randomize,
            flusher_queue_size=self.flusher_queue_size,
            no_echo=self.no_echo,
            tls=self.tls,
            tls_hostname=self.tls_hostname,
            user=self.user,
            password=self.password,
            token=self.token,
            drain_timeout=self.drain_timeout,
            signature_cb=self.signature_cb,
            user_jwt_cb=self.user_jwt_cb,
            user_credentials=self.user_credentials,
            nkeys_seed=self.nkeys_seed,
        )
        # Register Subscribers
        for topic_name in self.subscribers:
            self.event_loop.create_task(NATSSubscriber(self.nc, topic_name).run())

        # Register Publishers
        for topic_name in self.publishers:
            NATSPublisher(self.nc, topic_name, self.event_loop)

    async def close(self):
        print("NATS Connector: CLOSING CONNECTION")
        await self.nc.close()

    async def _disconnected_cb(self):
        print("NATS Connector: GOT DISCONNECTED")

    async def _reconnected_cb(self):
        print(f"NATS Connector: GOT RECONNECTED TO {self.nc.connected_url.netloc}")

    async def _error_cb(self, e):
        print(f"NATS Connector Error: {e}")

    async def _closed_cb(self):
        print("NATS Connector: CLOSED CONNECTION")
