#! /usr/bin/env python3
import rospy
import asyncio
from nats_ros_connector.nats_client import NATSClient


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


if __name__ == "__main__":
    ## Initialize node
    rospy.init_node("nats_connector")
    ## Parameters
    host = load_param("~host")
    publishers = load_param("~publishers")
    subscribers = load_param("~subscribers")
    # NATS Client
    nats_client = NATSClient(host, publishers, subscribers)
    asyncio.run(nats_client.run())
