#! /usr/bin/env python3
import rospy
import asyncio
from nats_ros_connector.nats_client import NATSClient


def load_param(name, default_value=None, is_required=False):
    if is_required:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, default_value)


async def await_shutdown(nats_client):
    """
    Sleeps while ROS is running to keep the event loop alive and
    closes the nats_client connection after ROS shuts down.
    """
    # Keep the event loop alive by sleeping
    while not rospy.is_shutdown():
        await asyncio.sleep(0.5)
    # After shutdown close connection
    await nats_client.close()


if __name__ == "__main__":
    # Initialize node
    rospy.init_node("nats_connector")
    # Parameters
    host = load_param("~host", is_required=True)
    # NATS Connection Params
    # See https://nats-io.github.io/nats.py/modules.html#asyncio-client
    name = load_param("~name", None)
    pedantic = load_param("~pedantic", False)
    verbose = load_param("~verbose", False)
    allow_reconnect = load_param("~allow_reconnect", True)
    connect_timeout = load_param("~connect_timeout", 2)
    reconnect_time_wait = load_param("~reconnect_time_wait", 2)
    max_reconnect_attempts = load_param("~max_reconnect_attempts", 60)
    ping_interval = load_param("~ping_interval", 120)
    max_outstanding_pings = load_param("~max_outstanding_pings", 2)
    dont_randomize = load_param("~dont_randomize", False)
    flusher_queue_size = load_param("~flusher_queue_size", 1024)
    no_echo = load_param("~no_echo", False)
    tls = load_param("~tls", None)
    tls_hostname = load_param("~tls_hostname", None)
    user = load_param("~user", None)
    password = load_param("~password", None)
    token = load_param("~token", None)
    drain_timeout = load_param("~drain_timeout", 30)
    signature_cb = load_param("~signature_cb", None)
    user_jwt_cb = load_param("~user_jwt_cb", None)
    user_credentials = load_param("~user_credentials", None)
    nkeys_seed = load_param("~nkeys_seed", None)
    # Publisher and Subscriber params
    publishers = load_param("~publishers", [])
    subscribers = load_param("~subscribers", [])
    services = load_param("~services", [])
    services_proxies = load_param("~service_proxies", [])
    srv_req_timeout = load_param("~srv_req_timeout", 1)
    # Create event loop
    event_loop = asyncio.get_event_loop()
    # NATS Client
    nats_client = NATSClient(
        host,
        publishers,
        subscribers,
        services,
        services_proxies,
        event_loop,
        name,
        pedantic,
        verbose,
        allow_reconnect,
        connect_timeout,
        reconnect_time_wait,
        max_reconnect_attempts,
        ping_interval,
        max_outstanding_pings,
        dont_randomize,
        flusher_queue_size,
        no_echo,
        tls,
        tls_hostname,
        user,
        password,
        token,
        drain_timeout,
        signature_cb,
        user_jwt_cb,
        user_credentials,
        nkeys_seed,
        srv_req_timeout
    )
    # Start NATS Client
    event_loop.create_task(nats_client.run())
    # Create shutdown task
    shut_down_task = event_loop.create_task(await_shutdown(nats_client))
    # Run the event loop until the shutdown task completes
    event_loop.run_until_complete(shut_down_task)
