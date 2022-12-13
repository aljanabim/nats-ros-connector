import asyncio
import nats
from nats_ros_connector.nats_publisher import NATSPublisher
from nats_ros_connector.nats_subscriber import NATSSubscriber


class NATSClient:
    def __init__(self, nats_host, publishers, subscribers):
        self.host = nats_host
        self.publishers = publishers
        self.subscribers = subscribers
        self.tasks = []

    async def run(self):
        self.nc = await nats.connect(
            self.host,
            error_cb=self._error_cb,
            reconnected_cb=self._reconnected_cb,
            disconnected_cb=self._disconnected_cb,
            closed_cb=self._closed_cb,
        )
        # Register Subscribers
        for subscriber in self.subscribers:
            sub_task = asyncio.create_task(
                NATSSubscriber(self.nc, subscriber["topic"], subscriber["type"]).run()
            )
            self.tasks.append(sub_task)

        # Register Publishers
        for publisher in self.publishers:
            pub_task = asyncio.create_task(
                NATSPublisher(self.nc, publisher["topic"], publisher["rate"]).run()
            )
            self.tasks.append(pub_task)

        await asyncio.wait(self.tasks)
        await self.nc.close()

    async def _disconnected_cb(self):
        print("Got disconnected!")

    async def _reconnected_cb(self):
        print(f"Got reconnected to {self.nc.connected_url.netloc}")

    async def _error_cb(self, e):
        print(f"There was an error: {e}")

    async def _closed_cb(self):
        print("Connection is closed")
